module Transformer
    # Implementation of an algorithm that propagates the frame information along
    # the dataflow network, and makes sure that frame selections are consistent.
    class FramePropagation < Syskit::NetworkGeneration::DataFlowComputation
        class FrameAnnotation
            attr_reader :task
            attr_reader :frame_name
            attr_reader :selected_frame

            def initialize(task, frame_name, selected_frame)
                @task, @frame_name, @selected_frame = task, frame_name, selected_frame
            end

            def empty?; !@selected_frame end
            def merge(ann)
                if !ann.kind_of?(FrameAnnotation)
                    raise ArgumentError, "cannot merge a frame annotation with a transform annotation. You are probably connecting two ports, one declared as a transform input or output and one only associated with a frame"
                end

                if ann.selected_frame != selected_frame
                    raise FrameSelectionConflict.new(task, frame_name, selected_frame, ann.selected_frame),
                        "invalid network: frame #{frame_name} in #{task} would need to select both #{ann.selected_frame} and #{selected_frame}"
                end
                false
            end

            def to_s
                "#<FrameAnnotation: #{task} #{frame_name}=#{selected_frame}>"
            end
        end

        # Assignment of a transformation during frame propagation
        class TransformAnnotation
            # The task on which we act
            attr_reader :task
            # The task's frame name for the source of the transformation
            attr_reader :from_frame
            # The selected source frame
            attr_reader :from
            # The task's frame name for the target of the transformation
            attr_reader :to_frame
            # The selected target frame
            attr_reader :to

            def initialize(task, from_frame, from, to_frame, to)
                @task = task
                @from_frame = from_frame
                @from = from
                @to_frame = to_frame
                @to   = to
            end

            # Needed by DataFlowComputation
            #
            # Returns true if neither +from+ nor +to+ are set
            def empty?; !@from && !@to end
            # Merge the information of two TransformAnnotation objects.
            #
            # This succeeds only if the two annotations point to the same
            # frames, or if one has nil and the other does not
            def merge(ann)
                if !ann.kind_of?(TransformAnnotation)
                    raise ArgumentError, "cannot merge a frame annotation with a transform annotation. You are probably connecting two ports, one declared as a transform input or output and one only associated with a frame"
                end

                changed = (@from != ann.from) || (@to != ann.to)
                @from ||= ann.from
                @to   ||= ann.to
                if ann.from && ann.from != from
                    raise FrameSelectionConflict.new(task, from_frame, from, ann.from), "incompatible selection: #{ann.from} != #{@from}"
                end
                if ann.to && ann.to != to
                    raise FrameSelectionConflict.new(task, to_frame, to, ann.to), "incompatible selection: #{ann.to} != #{@to}"
                end
                changed
            end

            def pretty_print(pp)
                pp.text "#{from} => #{to}"
            end

            def to_s # :nodoc:
                "#<TransformAnnotation: #{task} #{from_frame}=#{from} => #{to_frame}=#{to}>"
            end
        end

        def self.compute_frames(plan)
            algorithm = FramePropagation.new
            tasks = plan.find_local_tasks(Syskit::TaskContext).to_set
            algorithm.propagate(tasks)
        end

        def required_information(tasks)
            result = Hash.new
            tasks.each do |t|
                next if !(tr = t.model.transformer)

                task_info = [nil]
                tr.each_annotated_port do |port, frame|
                    task_info << port
                end
                result[t] = task_info
            end
            result
        end

        Trigger = Syskit::NetworkGeneration::DataFlowComputation::Trigger

        def triggering_port_connections(task)
            return if !(tr = task.model.transformer)

            interesting_ports = Set.new

            result = Hash.new
            connections = Set.new

            tr.each_annotated_port do |port, frame|
                interesting_ports << port.name
            end
            tr.each_transform_port do |port, transform|
                interesting_ports << port.name
            end

            interesting_ports.each do |port_name|
                task.each_concrete_input_connection(port_name) do |source_task, source_port, _|
                    connections << [source_task, source_port]
                end
                task.each_concrete_output_connection(port_name) do |_, sink_port, sink_task, _|
                    connections << [sink_task, sink_port]
                end

                if !connections.empty?
                    result[port_name] = Trigger.new(connections, Trigger::USE_PARTIAL)
                    connections = Set.new
                end
            end
            result
        end

        # This adds port information from device models only for ports that are
        # not associated with a transform. When there is a frame-to-port or
        # transform-to-port association registered, the device information is
        # added earlier in {initial_frame_selection_from_device}
        def propagate_device_information(task, dev)
            selected_frame = dev.frame
            selected_transform = dev.frame_transform
            tr = task.model.transformer

            task.find_all_driver_services_for(dev).each do |srv|
                srv.model.each_output_port do |out_port|
                    out_port = out_port.to_component_port
                    # Do not associate the ports that output transformations
                    if selected_transform && Transformer.transform_port?(out_port)
                        from, to = selected_transform.from, selected_transform.to
                        if !tr || !tr.find_transform_of_port(out_port.name)
                            Transformer.debug do
                                Transformer.debug "directly associating transform #{from} => #{to} to port #{out_port} using information from device #{dev.full_name} driven by #{srv.name}"
                                break
                            end
                            add_port_info(task, out_port.name,
                                TransformAnnotation.new(task, nil, from, nil, to))
                            done_port_info(task, out_port.name)
                        end
                    elsif selected_frame
                        if !tr || !tr.find_frame_of_port(out_port.name)
                            Transformer.debug do
                                Transformer.debug "directly associating frame #{selected_frame} to port #{out_port} using information from device #{dev.full_name} driven by #{srv.name}"
                                break
                            end
                            add_port_info(task, out_port.name,
                                FrameAnnotation.new(task, selected_frame, selected_frame))
                            done_port_info(task, out_port.name)
                        end
                    end
                end
            end

            if dev.respond_to?(:each_slave)
                dev.each_slave do |slave_dev|
                    propagate_device_information(task, slave_dev)
                end
            end
        end

        def initial_information(task)
            # Add frame information from the devices if there is some
            # This does not require the presence of a transformer spec
            if task.respond_to?(:each_master_device)
                task.each_master_device do |dev|
                    propagate_device_information(task, dev)
                end
            end

            # Now look for transformer-specific information
            tr = task.model.transformer
            return if !tr

            # Now add information for all ports for which we know the frame
            # already
            tr.each_annotated_port do |port, frame_name|
                if selected_frame = task.selected_frames[frame_name]
                    add_port_info(task, port.name, FrameAnnotation.new(task, frame_name, selected_frame))
                    done_port_info(task, port.name)
                end
            end
            tr.each_transform_port do |port, transform|
                from = task.selected_frames[transform.from]
                to   = task.selected_frames[transform.to]
                add_port_info(task, port.name, TransformAnnotation.new(task, transform.from, from, transform.to, to))
                if from && to
                    done_port_info(task, port.name)
                end
            end

            Transformer.debug do
                Transformer.debug "initially selected frames for #{task}"
                available_frames = task.model.transformer.available_frames.dup
                task.selected_frames.each do |frame_name, selected_frame|
                    Transformer.debug "  selected #{selected_frame} for #{frame_name}"
                    available_frames.delete(frame_name)
                end
                Transformer.debug "  #{available_frames.size} frames left to pick: #{available_frames.to_a.sort.join(", ")}"
                break
            end
        end

        class PortAssociationMismatch < RuntimeError
            # The problematic endpoint, as a [task, port_name] pair
            attr_reader :endpoint
            # The other side of the problematic connection(s) 
            attr_reader :connections
            # The association type expected by +endpoint+. Can either be 'frame'
            # for an association between a port and a frame, and 'transform' for
            # an association between a port and a transformation.
            attr_reader :association_type

            def initialize(task, port_name, type)
                @endpoint = [task, port_name]
                @association_type = type

                @connections = []
                task.each_concrete_input_connection(port_name) do |source_task, source_port_name, _|
                    @connections << [source_task, source_port_name]
                end
                task.each_concrete_output_connection(port_name) do |_, sink_port_name, sink_task, _|
                    @connections << [sink_task, sink_port_name]
                end
            end

            def pretty_print(pp)
                pp.text "#{endpoint[0]}.#{endpoint[1]} was expecting an association with a #{association_type}, but one or more connections mismatch"
                pp.nest(2) do
                    pp.breakable
                    pp.seplist(connections) do |conn|
                        pp.text "#{conn[0]}.#{conn[1]}"
                    end
                end
            end
        end

        def propagate_task(task)
            return if !(tr = task.model.transformer)

            # First, save the port annotations into the select_frames hash on
            # the task.
            tr.each_annotated_port do |port, frame|
                if has_information_for_port?(task, port.name)
                    info = port_info(task, port.name)
                    if !info.respond_to?(:selected_frame)
                        raise PortAssociationMismatch.new(task, port.name, 'frame')
                    end

                    debug { "selecting #{info.selected_frame} on #{task} for #{frame} through port #{port.name}" }
                    task.select_frame(frame, info.selected_frame)
                end
            end
            tr.each_transform_port do |port, transform|
                next if !has_information_for_port?(task, port.name)
                info = port_info(task, port.name)

                if info.from
                    debug { "selecting #{info.from} on #{task} for #{transform.from} through the source frame of port #{port.name}" }
                    task.select_frame(transform.from, info.from)
                end
                if info.to
                    debug { "selecting #{info.to} on #{task} for #{transform.to} through the source frame of port #{port.name}" }
                    task.select_frame(transform.to, info.to)
                end
            end

            # Then propagate newly found information to ports that are
            # associated with the frames
            has_all = true
            tr.each_annotated_port do |port, frame_name|
                next if has_final_information_for_port?(task, port.name)

                if selected_frame = task.selected_frames[frame_name]
                    if !has_information_for_port?(task, port.name)
                        add_port_info(task, port.name, FrameAnnotation.new(task, frame_name, selected_frame))
                        done_port_info(task, port.name)
                    end
                else
                    has_all = false
                end
            end
            tr.each_transform_port do |port, transform|
                next if has_final_information_for_port?(task, port.name)
                from = task.selected_frames[transform.from]
                to   = task.selected_frames[transform.to]
                add_port_info(task, port.name, TransformAnnotation.new(task, transform.from, from, transform.to, to))
                if from && to
                    done_port_info(task, port.name)
                else
                    has_all = false
                end
            end
            return has_all
        end

        # Given a task and a device, map the frame or frame transform
        # information of the device towards frame information on the task
        #
        # The mapping is done through ports, i.e. it requires frame-to-port
        # and/or transform-to-port information to be set properly on the task
        # model. For ports that do not have this information, the frame
        # configuration stored in the device model is propagated during the
        # frame propagation pass by {propagate_device_information}
        #
        # @param [TaskContext] task
        # @param [DeviceInstance] dev
        # @returns [Hash<String,String>] the frame mappings, from a task local
        #   frame to a global frame name
        def self.initial_frame_selection_from_device(task, dev)
            tr = task.model.transformer
            if selected_frame = dev.frame
                if !task.transformer.has_frame?(selected_frame)
                    raise Transformer::InvalidConfiguration, "undefined frame #{selected_frame} selected as reference frame for #{dev}"
                end
            end
            if selected_transform = dev.frame_transform
                if !task.transformer.has_frame?(selected_transform.from)
                    raise Transformer::InvalidConfiguration, "undefined frame #{selected_transform.from} selected as 'from' frame for #{dev}"
                end
                if !task.transformer.has_frame?(selected_transform.to)
                    raise Transformer::InvalidConfiguration, "undefined frame #{selected_transform.to} selected as 'to' frame for #{dev}"
                end
            end

            new_selections = Hash.new
            task.find_all_driver_services_for(dev).each do |srv|
                srv.model.each_output_port do |out_port|
                    out_port = out_port.to_component_port

                    if selected_transform && Transformer.transform_port?(out_port)
                        from, to = selected_transform.from, selected_transform.to
                        if transform = tr.find_transform_of_port(out_port.name)
                            if from
                                Transformer.debug { "selecting #{from} for #{transform.from} on #{task}, using information from device #{dev}" }
                                new_selections[transform.from] = from
                            end
                            if to
                                Transformer.debug { "selecting #{to} for #{transform.to} on #{task}, using information from device #{dev}" }
                                new_selections[transform.to] = to
                            end
                        end
                    elsif selected_frame
                        if frame_name = tr.find_frame_of_port(out_port.name)
                            Transformer.debug { "selecting #{selected_frame} for #{frame_name} on #{task}, using information from device #{dev}" }
                            new_selections[frame_name] = selected_frame
                        end
                    end
                end
            end

            if dev.respond_to?(:each_slave)
                dev.each_slave do |slave_dev|
                    new_selections.merge!(initial_frame_selection_from_device(task, slave_dev)) do |frame, dev_frame, slave_frame|
                        raise FrameSelectionConflict.new(task, frame, dev_frame, slave_frame), "mismatch between in selection of frame #{frame} in #{task}: device #{dev} requires #{dev_frame} and its slave #{slave_dev.full_name} requires #{slave_frame}"
                    end
                end
            end
            new_selections
        end

        def self.initialize_selected_frames(task, current_selection)
            tr = if task.model.respond_to?(:transformer)
                     task.model.transformer
                 end

            new_selections = if task.requirements
                                 task.requirements.frame_mappings
                             else
                                 Hash.new
                             end

            static_frames = Hash.new

            # Do selection for the frames that can't be configured anyways
            if tr
                if task.respond_to?(:each_master_device)
                    task.each_master_device do |dev|
                        device_selections = initial_frame_selection_from_device(task, dev)
                        debug do
                            debug "selecting frames on #{task} from #{dev}"
                            debug "  #{device_selections}"
                        end
                        new_selections.merge!(device_selections) do |frame, req_sel, dev_sel|
                            if req_sel != dev_sel
                                raise FrameSelectionConflict.new(task, frame, req_sel, dev_sel), "mismatch between selected frame #{req_sel} and device frame #{dev_sel} from #{dev.name}"
                            end
                            req_sel
                        end
                    end
                end
                tr.each_statically_mapped_frame do |frame_name|
                    debug { "selected frame #{frame_name} on #{task} for #{frame_name}: static frame" }
                    if (sel = new_selections[frame_name]) && (sel != frame_name)
                        raise StaticFrameChangeError.new(task, frame_name, sel)
                    end
                    static_frames[frame_name] = frame_name
                end
            end

            result =
                if new_selections.empty?
                    debug { "selecting frame mappings #{current_selection} on #{task}, propagated from its parents" }
                    current_selection
                else
                    debug { "adding frame selection from #{task}: #{new_selections}" }
                    new_selections.each do |frame_name, selected_frame|
                        if !task.transformer.has_frame?(selected_frame)
                            raise InvalidConfiguration, "undefined frame #{selected_frame} selected for '#{frame_name}' in #{task}"
                        end
                    end
                    new_selections
                end
            result = result.merge(static_frames)
            task.select_frames(result)
            result
        end

        def self.initialize_transform_producers(task, current_selection)
            new_info =
                if task.requirements
                    current_selection.dup.merge(task.requirements.transformer)
                else
                    current_selection.dup
                end
            task.transformer.merge(new_info)
        end
    end
end

