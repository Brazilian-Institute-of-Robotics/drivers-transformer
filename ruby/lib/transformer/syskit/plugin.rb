module Transformer
    module SyskitPlugin
        def self.compute_required_transformations(manager, task)
            static_transforms  = Hash.new
            dynamic_transforms = Hash.new { |h, k| h[k] = Array.new }

            tr = task.model.transformer
            if !tr
                return static_transforms, dynamic_transforms
            end

            tr.each_needed_transformation do |trsf|
                from = task.selected_frames[trsf.from]
                to   = task.selected_frames[trsf.to]
                if !from || !to
                    # This is validated in #validate_generated_network. Just
                    # ignore here.
                    next
                end
                
                # Register transformation producers that are connected to
                # some of our transformation input ports
                self_producers = Hash.new
                tr.each_transform_port do |port, transform|
                    if port.kind_of?(Orocos::Spec::InputPort) && task.connected?(port.name)
                        port_from = task.selected_frames[transform.from]
                        port_to   = task.selected_frames[transform.to]
                        if port_from && port_to
                            self_producers[[port_from, port_to]] = port
                        end
                    end
                end
                # Special case: dynamic_transformations
                if dyn_port = task.find_port('dynamic_transformations')
                    dyn_port.each_concrete_connection do |out_port, _|
                        transform = out_port.produced_transformation
                        if transform && transform.from && transform.to
                            self_producers[[transform.from, transform.to]] = out_port.model.orogen_model
                        end
                    end
                end

                Transformer.debug do
                    Transformer.debug "looking for chain for #{from} => #{to} in #{task}"
                    Transformer.debug "  with local producers: #{self_producers}"
                    break
                end
                chain =
                    begin
                        manager.transformation_chain(from, to, self_producers)
                    rescue Exception => e
                        raise InvalidChain.new(task, trsf.from, from, trsf.to, to, e),
                            "cannot find a transformation chain to produce #{from} => #{to} for #{task} (task-local frames: #{trsf.from} => #{trsf.to}): #{e.message}", e.backtrace
                    end
                Transformer.log_pp(:debug, chain)

                static, dynamic = chain.partition
                Transformer.debug do
                    Transformer.debug "#{static.to_a.size} static transformations"
                    Transformer.debug "#{dynamic.to_a.size} dynamic transformations"
                    break
                end

                static.each do |trsf|
                    static_transforms[[trsf.from, trsf.to]] = trsf
                end
                dynamic.each do |dyn|
                    # If we find a producer that is an input port, it means
                    # that the task is already explicitly connected to this
                    # producer (injected in the self_producers hash above).
                    # Just ignore it here, we don't need to instanciate it
                    # ourselves
                    next if dyn.producer.kind_of?(Orocos::Spec::Port)
                    dynamic_transforms[dyn.producer] << dyn
                end
            end
            return static_transforms, dynamic_transforms
        end

        def self.instanciate_producer(manager, task, producer_model, transformations)
            needed_transformations = transformations.find_all do |dyn|
                role_name = "transformer_#{dyn.from}2#{dyn.to}"
                !task.find_child_from_role(role_name)
            end
            return if needed_transformations.empty?

            producer = producer_model.instanciate(task.plan)
            producer_task = producer.to_task
            producer_task.transformer.merge(manager.conf)
            propagate_local_transformer_configuration(producer_task)

            Transformer.debug { "instanciated #{producer_task} for #{task}" }
            transformations.each do |dyn|
                Transformer.debug { "adding #{producer_task} as child transformer_#{dyn.from}2#{dyn.to} of #{task}" }
                task.depends_on(producer_task, :role => "transformer_#{dyn.from}2#{dyn.to}")

                out_port = producer.find_port_for_transform(dyn.from, dyn.to)
                if !out_port
                    if out_port = producer_task.find_port_for_transform(dyn.from, dyn.to)
                        producer = producer_task
                    else
                        raise TransformationPortNotFound.new(producer_task, dyn.from, dyn.to)
                    end
                end

                in_ports = task.find_all_input_ports_for_transform(dyn.from, dyn.to)
                begin
                    producer.select_port_for_transform(out_port, dyn.from, dyn.to)
                rescue InvalidFrameSelection => e
                    e.producer_for << task
                    raise
                end

                if dyn_in_port = task.find_port('dynamic_transformations')
                    in_ports << dyn_in_port
                end
                in_ports.each do |p|
                    out_port.connect_to p
                end
            end

            # Manually propagate device information on the new task
            if producer_task.respond_to?(:each_master_device) && producer_task.model.transformer
                producer_task.each_master_device do |dev|
                    device_frames = FramePropagation.
                        initial_frame_selection_from_device(producer_task, dev)
                    begin
                        producer_task.select_frames(device_frames)
                    rescue FrameSelectionConflict => e
                        raise e, "#{e.message}: conflict between frame #{e.current_frame} selected for #{e.frame} and frame #{e.new_frame} from device #{dev.name}", e.backtrace
                    end
                end
            end
            producer_task
        end

        # Adds the transformation producers needed to properly setup the system.
        #
        # @return [Boolean] true if producers have been added to the plan and
        #   false otherwise
        def self.add_needed_producers(tasks, instanciated_producers)
            has_new_producers = false
            tasks.each do |task|
                tr_config = task.transformer
                tr_manager = Transformer::TransformationManager.new(tr_config)

                Transformer.debug { "computing needed static and dynamic transformations for #{task}" }

                static_transforms, dynamic_transforms = compute_required_transformations(tr_manager, task)
                task.static_transforms = static_transforms.values
                dynamic_transforms.each do |producer_model, transformations|
                    producer_tasks = instanciated_producers[producer_model]
                    if !producer_tasks.empty?
                        is_recursive = producer_tasks.any? do |prod_task|
                            prod_task == task || prod_task.reachable?(task, Roby::TaskStructure::Dependency)
                        end
                        if is_recursive
                            raise RecursiveProducer, "#{producer_model} requires some transformations (#{transformations.map { |tr| "#{tr.from}=>#{tr.to}" }}) that are produced by itself"
                        end
                    end

                    if producer_task = instanciate_producer(tr_manager, task, producer_model, transformations)
                        has_new_producers = true
                        instanciated_producers[producer_model] << producer_task
                    end
                end
            end
            has_new_producers
        end

        def self.update_configuration_state(state, tasks)
            state.port_transformation_associations.clear
            state.port_frame_associations.clear

            static_transforms = Hash.new

            tasks.each do |task|
                task.requirements.transformer.each_static_transform do |static|
                    static_transforms[[static.from, static.to]] = static
                end
                
                tr = task.model.transformer
                task_name = task.orocos_name
                tr.each_annotated_port do |port, frame_name|
                    selected_frame = task.selected_frames[frame_name]
                    if selected_frame
                        info = Types::Transformer::PortFrameAssociation.new(
                            :task => task_name, :port => port.name, :frame => selected_frame)
                        state.port_frame_associations << info
                    elsif Syskit.conf.transformer_warn_about_unset_frames?
                        Transformer.warn "no frame selected for #{frame_name} on #{task}. This is harmless for the network to run, but will make the display of #{port.name} \"in the right frame\" impossible"
                    end
                end
                tr.each_transform_port do |port, transform|
                    next if port.kind_of?(Orocos::Spec::InputPort)

                    from = task.selected_frames[transform.from]
                    to   = task.selected_frames[transform.to]
                    if from && to
                        info = Types::Transformer::PortTransformationAssociation.new(
                            :task => task_name, :port => port.name,
                            :from_frame => from, :to_frame => to)
                        state.port_transformation_associations << info
                    elsif Syskit.conf.transformer_warn_about_unset_frames?
                        Transformer.warn "no frame selected for #{transform.to} on #{task}. This is harmless for the network to run, but might remove some options during display"
                    end
                end
            end

            state.static_transformations = static_transforms.values.map do |static|
                rbs = Types::Base::Samples::RigidBodyState.invalid
                rbs.sourceFrame = static.from
                rbs.targetFrame = static.to
                rbs.position = static.translation
                rbs.orientation = static.rotation
                rbs
            end
        end

        def self.instanciation_postprocessing_hook(engine, plan)
            # Transfer the frame mapping information from the instance specification
            # objects to the selected_frames hashes on the tasks
            tasks = plan.find_local_tasks(Syskit::Component).roots(Roby::TaskStructure::Dependency)
            tasks.each do |root_task|
                propagate_local_transformer_configuration(root_task)
            end
        end

        class TransformerConfigurationVisitor < RGL::DFSVisitor
            attr_reader :selected_frames

            def handle_start_vertex(root_task)
                @selected_frames = Hash.new
                selected_frames[root_task] = FramePropagation.initialize_selected_frames(root_task, Hash.new)
                FramePropagation.initialize_transform_producers(root_task, Transformer::Configuration.new)
            end

            def handle_examine_edge(from, to)
                selected_frames[to] = FramePropagation.initialize_selected_frames(
                    to, selected_frames[from])
                FramePropagation.initialize_transform_producers(to, from.transformer)
            end
        end

        def self.propagate_local_transformer_configuration(root_task)
            dependency_graph = root_task.relation_graph_for(Roby::TaskStructure::Dependency)
            visitor = TransformerConfigurationVisitor.new(dependency_graph)
            visitor.handle_start_vertex(root_task)
            dependency_graph.depth_first_visit(root_task, visitor) {}
        end

        # Validate that the frame selected for the given task are consistent
        # with its connections
        #
        # It only checks its inputs, as it is meant to iterate over all tasks
        def self.instanciated_network_postprocessing_hook(engine, plan)
            needed = true
            all_producers = Hash.new { |h, k| h[k] = Array.new }
            while needed
                FramePropagation.compute_frames(plan)
                transformer_tasks = plan.find_local_tasks(Syskit::TaskContext).
                    find_all { |task| task.model.transformer }

                # Now find out the frame producers that each task needs, and add them to
                # the graph
                needed = add_needed_producers(transformer_tasks, all_producers)
            end
        end

        def self.deployment_postprocessing_hook(engine, plan)
            transformer_tasks = plan.find_local_tasks(Syskit::TaskContext).
                find_all { |task| task.model.transformer }

            # And update the configuration state
            update_configuration_state(plan.transformer_configuration_state[1], transformer_tasks)
            plan.transformer_configuration_state[0] = Time.now
        end

        module RobyAppPlugin
            def self.setup(app)
                if app.testing?
                    Syskit.conf.transformer_warn_about_unset_frames = false
                end

                Roby.app.using_task_library('transformer')
            end

            def self.require_config(app)
                Syskit.conf.use_deployment('transformer_broadcaster')
            end
        end

        def self.enable
            Roby.app.add_plugin 'syskit-transformer', RobyAppPlugin

            # Maintain a transformer broadcaster on the main engine
            Roby::ExecutionEngine.add_propagation_handler(description: 'syskit-transformer transformer broadcaster start') do |plan|
		if Syskit.conf.transformer_broadcaster_enabled?
		    if !plan.execution_engine.quitting? && plan.find_tasks(OroGen::Transformer::Task).not_finished.empty?
			plan.add_mission_task(OroGen::Transformer::Task)
		    end
		end
            end

            Syskit::NetworkGeneration::Engine.register_instanciation_postprocessing do |engine, plan|
                if Syskit.conf.transformer_enabled?
                    instanciation_postprocessing_hook(engine, plan)
                end
            end

            Syskit::NetworkGeneration::Engine.register_instanciated_network_postprocessing do |engine, plan|
                if Syskit.conf.transformer_enabled?
                    instanciated_network_postprocessing_hook(engine, plan)
                end
            end

            Syskit::NetworkGeneration::Engine.register_deployment_postprocessing do |engine, plan|
                if Syskit.conf.transformer_enabled?
                    deployment_postprocessing_hook(engine, plan)
                end
            end

            Syskit::Component.class_eval do
                prepend Transformer::ComponentExtension
                extend Transformer::ComponentModelExtension
            end
            Syskit::TaskContext.class_eval do
                prepend Transformer::TaskContextExtension
                extend Transformer::TaskContextModelExtension
            end
            Syskit::Port.class_eval do
                prepend Transformer::PortExtension
            end
            Syskit::Composition.class_eval do
                prepend Transformer::CompositionExtension
            end
            Syskit::BoundDataService.class_eval do
                prepend Transformer::BoundDataServiceExtension
            end
            Roby::Plan.class_eval do
                prepend Transformer::PlanExtension
            end

            Syskit::Robot::DeviceInstance.class_eval do
                prepend Transformer::DeviceExtension
            end
            Syskit::Graphviz.class_eval do
                prepend Transformer::GraphvizExtension
            end
            Syskit::Graphviz.available_task_annotations << 'transforms'
            Syskit::InstanceRequirements.class_eval do
                prepend Transformer::InstanceRequirementsExtension
            end
            Syskit::NetworkGeneration::SystemNetworkGenerator.class_eval do
                prepend Transformer::SystemNetworkGeneratorExtension
            end
            Syskit::Actions::Profile.class_eval do
                prepend Transformer::ProfileExtension
            end
        end

        def self.register
            Syskit::RobyApp::Configuration.class_eval do
                prepend Transformer::ConfigurationExtension
            end
            Syskit::TaskContext.extend Transformer::TransformerConfigurationAccess
            Roby.app.filter_out_patterns.push(/^#{Regexp.quote(__FILE__)}/)
        end
    end
end

