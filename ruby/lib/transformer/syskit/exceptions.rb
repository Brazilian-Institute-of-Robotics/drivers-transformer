module Transformer
    # Exception raised when a frame is being selected with #selected_frame, but
    # the selection is invalid
    #
    # The reason for the invalidity is (for now) stored only in the message
    class InvalidFrameSelection < RuntimeError
        # The task for which the selection was done
        attr_reader :task
        # The task's frame name
        attr_reader :frame
        # List of [task, port] pairs that give information about why we are
        # updating the frame
        attr_reader :related_ports
        # List of dynamic transformations, and their recipients, that 'task' is
        # used as a producer for
        attr_reader :producer_for

        def initialize(task, frame)
            @task, @frame = task, frame
            @related_ports = compute_related_ports
            @producer_for  = Array.new
        end

        def compute_related_ports
            return Array.new if !(trsf = task.model.transformer)

            result = Array.new
            task.model.each_annotated_port do |port, frame_name|
                next if frame_name != self.frame
                port.bind(task).each_frame_of_connected_ports do |other_port, other_frame|
                    result << [other_port, other_frame]
                end
            end
            task.model.each_transform_port do |port, transform|
                if self.frame == transform.from
                    port.bind(task).each_transform_of_connected_ports do |other_port, other_transform|
                        if other_transform.from
                            result << [other_port, other_transform.from]
                        end
                    end
                elsif self.frame == transform.to
                    port.bind(task).each_transform_of_connected_ports do |other_port, other_transform|
                        if other_transform.to
                            result << [other_port, other_transform.to]
                        end
                    end
                end
            end
            result
        end

        def pretty_print_common_info(pp)
            pp.breakable
            pp.text "Transformer is configured as follows:"
            pp.nest(2) do
                pp.breakable
                task.transformer.pretty_print(pp)
            end
            if !task.requirements.frame_mappings.empty?
                pp.breakable
                pp.text "Requirements specify:"
                pp.nest(2) do
                    pp.breakable
                    pp.seplist(task.requirements.frame_mappings) do |mapping|
                        local_name, global_name = *mapping
                        pp.text "#{local_name}: #{global_name}"
                    end
                end
            end
            if !related_ports.empty?
                pp.breakable
                pp.text "Related ports:"
                pp.nest(2) do
                    pp.breakable
                    pp.seplist(related_ports) do |src|
                        task, port_name, info = *src
                        pp.text "#{src[0]}: #{src[1]}"
                        if info
                            pp.text " (#{info})"
                        end
                    end
                end
            end
            if !producer_for.empty?
                pp.breakable
                pp.text "The task is being used as a producer for:"
                pp.nest(2) do
                    pp.breakable
                    pp.seplist(producer_for) do |task|
                        task.pretty_print(pp)
                    end
                end
            end
        end
    end

    # Exception raised when two different frames are being selected for the same
    # task/frame_name pair
    class FrameSelectionConflict < InvalidFrameSelection
        # The currently selected frame
        attr_reader :current_frame
        # The newly selected frame
        attr_reader :new_frame

        def initialize(task, frame, current, new)
            super(task, frame)
            @current_frame = current
            @new_frame = new
        end

        def pretty_print(pp)
            pp.text "conflicting frames selected for #{frame} (#{current_frame} != #{new_frame})"
            pp.breakable
            pp.text "on "
            task.pretty_print(pp)
            pretty_print_common_info(pp)
        end
    end

    # Exception thrown in #merge if the merged task has a different frame
    # selection than the merging task
    class FrameSelectionConflictDuringMerge < FrameSelectionConflict
        # The task that was being merged
        attr_reader :merged_task

        def initialize(merging_task, merged_task, frame, current, new)
            super(merging_task, frame, current, new)
            @merged_task = merged_task
        end

        def pretty_print(pp)
            pp.text "while doing #{task}.merge(#{merged_task})"
            pp.breakable
            super
        end
    end

    # Exception raised when #select_frame is called on a static frame with a
    # different name than the frame's static name
    class StaticFrameChangeError < InvalidFrameSelection
        # The name of the frame that was being assigned to a static frame
        attr_reader :new_frame
        def initialize(task, frame, new)
            super(task, frame)
            @new_frame = new
        end
        def pretty_print(pp)
            pp.text "cannot change frame #{task}.#{frame} to #{new_frame}, as the component does not support it"
            if !related_ports.empty?
                pp.text "related ports:"
                pp.nest(2) do
                    pp.breakable
                    pretty_print_related_ports(pp)
                end
            end
        end
    end

    # Exception raised during network generation if the system cannot find a
    # production chain for a transformation
    class InvalidChain < RuntimeError
        # The task for which the transformation was being produced
        attr_reader :task
        # The task-local name for the source frame
        attr_reader :task_from
        # The task-local name for the target frame
        attr_reader :task_to
        # The global name for the source frame
        attr_reader :from
        # The global name for the target frame
        attr_reader :to
        # The exception explaining the error
        attr_reader :reason

        def initialize(task, task_from, from, task_to, to, reason)
            @task, @task_from, @from, @task_to, @to, @reason =
                task, task_from, from, task_to, to, reason
        end

        def pretty_print(pp)
            pp.text "cannot find a production chain for #{from} => #{to}"
            pp.breakable
            pp.text "  in #{task}"
            pp.breakable
            pp.text "  (task-local: #{task_from} => #{task_to})"
            pp.breakable
            reason.pretty_print(pp)
        end
    end

    # Exception raised during network generation if a declared producer cannot
    # provide the required transformation
    class TransformationPortNotFound < RuntimeError
        attr_reader :task
        attr_reader :from
        attr_reader :to

        def initialize(task, from, to)
            @task, @from, @to = task, from, to
        end

        def pretty_print(pp)
            pp.text "cannot find a port providing the transformation #{from} => #{to} on"
            pp.breakable
            task.pretty_print(pp)
        end
    end
    # Exception raised during network generation if multiple ports can provide
    # a required transformation
    class TransformationPortAmbiguity < RuntimeError
        attr_reader :task
        attr_reader :from
        attr_reader :to
        attr_reader :candidates

        def initialize(task, from, to, candidates)
            @task, @from, @to, @candidates = task, from, to, candidates
        end

        def pretty_print(pp)
            pp.text "multiple candidate ports to provide the transformation #{from} => #{to} on"
            pp.nest(2) do
                pp.breakable
                task.pretty_print(pp)
            end
            pp.breakable
            pp.text "Candidates:"
            pp.nest(2) do
                pp.breakable
                pp.seplist(candidates) do |c|
                    c.pretty_print(pp)
                end
            end
        end
    end

    # Exception raised when a needed frame is not assigned
    class MissingFrame < RuntimeError; end

    # Exception raised when a producer requires itself to function
    class RecursiveProducer < RuntimeError; end

end

