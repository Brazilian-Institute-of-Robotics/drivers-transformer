module Transformer
    # Module used to extend objects of the class Syskit::Component
    module ComponentExtension
        attribute(:selected_frames) { Hash.new }
        attribute(:transformer) { Transformer::Configuration.new }

        def find_transform_of_port(port)
        end

        def find_frame_of_port(port)
        end

        def find_port_for_transform(from, to)
        end

        def can_merge?(other)
            if !super
                return false
            end

            selected_frames.each do |task_frame, selected_frame|
                if other_sel = other.selected_frames[task_frame]
                    if other_sel != selected_frame
                        return false
                    end
                end
            end

            transformer.compatible_with?(other.transformer)
            return true
        end

        def merge(merged_task)
            transformer.merge(merged_task.transformer)
            super if defined? super
        end

        def select_frame(frame_name, selected_frame, validate: true)
            raise NotImplementedError, "cannot select frames on a composition"
        end

        def select_frames(mappings)
        end

        # Returns true if the specified transformation is provided through a
        # dedicated port on the task, or if it should be built by the
        # transformer by aggregating information from dynamic_transformations
        #
        # The frame names are actual frame names, not task-local ones
        def find_transformation_input(from, to)
            return if !(tr = model.transformer)
            tr.each_transform_input do |port, transform|
                port_from = selected_frames[transform.from]
                port_to   = selected_frames[transform.to]
                if port_from == from && port_to == to
                    return port
                end
            end
            nil
        end

        # Yields the ports for which a frame is associated, as well as the frame
        # name
        #
        # @yieldparam [Syskit::Port] the port
        # @yieldparam [String,nil] the frame name. This is a global name, not a
        #   task-local one. It is nil if the port is associated to a frame at
        #   model level, but this frame is not yet assigned
        def each_annotated_port
            return enum_for(:each_annotated_port) if !block_given?
            model.each_annotated_port do |port, frame_name|
                yield port.bind(self), selected_frames[frame_name]
            end
        end

        # Yields the ports for which a transformation is associated, as well as
        # the frame name
        #
        # @yieldparam [Syskit::Port] the port
        # @yieldparam [Transform] the associated transformation. This uses
        #   global names, not task-local ones. 'from', 'to' or both can be nil if
        #   some of the frames are not yet assigned at the task level
        def each_transform_port
            return enum_for(:each_transform_port) if !block_given?
            model.each_transform_port do |port, transform|
                from = selected_frames[transform.from]
                to   = selected_frames[transform.to]
                yield port.bind(self), Transform.new(from, to)
            end
        end

        # Returns true if one of the task's input port is configured to provide
        # the requested transformation
        def has_transformation_input?(from, to)
            !!find_transformation_input(from, to)
        end
    end
end

