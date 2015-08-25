module Transformer
    # Module that extends the TaskContext class itself
    module TaskContextModelExtension
        def each_transform_input
            return enum_for(:each_transform_input) if !block_given?
            return if !(tr = transformer)

            each_input_port do |port|
                if associated_transform = tr.find_transform_of_port(port.name)
                    yield(port, associated_transform.from, associated_transform.to)
                end
            end
        end

        def each_transform_output
            return enum_for(:each_transform_output) if !block_given?
            return if !(tr = transformer)

            each_output_port do |port|
                if associated_transform = tr.find_transform_of_port(port.name)
                    yield(port, associated_transform.from, associated_transform.to)
                end
            end
        end

        # Returns the transformation that this port provides, using the
        # task-local names
        def find_transform_of_port(port)
            return if !(tr = transformer)
            tr.find_transform_of_port(port.name)
        end

        # Returns the frame in which this port's data is expressed, using global
        # names, not task-local ones
        #
        # @return [String,nil] returns the frame name, or nil if either this
        #   port is not associated to any frame, or if this frame is not yet
        #   selected
        def find_frame_of_port(port)
            return if !(tr = transformer)
            tr.find_frame_of_port(port.name)
        end
    end
end

