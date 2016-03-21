module Transformer
    # Module used to add some functionality to Syskit::NetworkGeneration::Engine
    module SystemNetworkGeneratorExtension
        # During network validation, checks that all required frames have been
        # configured
        def validate_generated_network(plan, options)
            super

            if Syskit.conf.transformer_enabled?
                plan.find_local_tasks(Syskit::TaskContext).each do |task|
                    next if !(tr = task.model.transformer)

                    tr.each_needed_transformation do |transform|
                        if !task.selected_frames[transform.from]
                            raise MissingFrame, "could not find a frame assignment for #{transform.from} in #{task}"
                        end
                        if !task.selected_frames[transform.to]
                            raise MissingFrame, "could not find a frame assignment for #{transform.to} in #{task}"
                        end
                    end
                end
            end
        end


        def validate_abstract_network
            super

            # We must now validate. The frame propagation algorithm does
            # some validation, but also tries to do as little work as
            # possible and therefore will miss some errors
            if engine.options[:validate_abstract_network]
                transformer_tasks = plan.find_local_tasks(Syskit::TaskContext).
                    find_all { |task| task.model.transformer }
                transformer_tasks.each do |task|
                    SystemNetworkGeneratorExtension.validate_frame_selection_consistency_through_inputs(task)
                end
            end
        end

        def self.validate_frame_selection_consistency_through_inputs(task)
            task.each_annotated_port do |task_port, task_frame|
                next if !task_port.input? || !task_frame
                task_port.each_frame_of_connected_ports do |other_port, other_frame|
                    if other_frame != task_frame
                        raise FrameSelectionConflict.new(
                            task,
                            task.model.find_frame_of_port(task_port),
                            task_frame,
                            other_frame)
                    end
                end
            end
            task.each_transform_port do |task_port, task_transform|
                next if !task_port.input?
                task_port.each_transform_of_connected_ports do |other_port, other_transform|
                    if other_transform.from && task_transform.from && other_transform.from != task_transform.from
                        task_local_name = task.model.find_transform_of_port(task_port).from
                        raise FrameSelectionConflict.new(task, task_local_name,
                                                         task_transform.from, other_transform.from)
                    elsif other_transform.to && task_transform.to && other_transform.to != task_transform.to
                        task_local_name = task.model.find_transform_of_port(task_port).to
                        raise FrameSelectionConflict.new(task, task_local_name,
                                                         task_transform.to, other_transform.to)
                    end
                end
            end
        end
    end
end

