module Transformer
    module CompositionExtension
        # Returns an output port object that is providing the requested
        # transformation, or nil if none can be found
        #
        # Raises TransformationPortAmbiguity if multiple ports match.
        def find_port_for_transform(from, to)
            associated_candidates = []
            type_candidates = []
            each_output_port do |port|
                next if !Transformer.transform_port?(port)

                if transform = find_transform_of_port(port)
                    if transform.from == from && transform.to == to
                        return port
                    elsif ((transform.from == from || !transform.from) && (transform.to == to || !transform.to))
                        associated_candidates << port
                    end
                else
                    type_candidates << port
                end
            end

            if associated_candidates.size == 1
                return associated_candidates.first
            elsif associated_candidates.size > 1
                raise TransformationPortAmbiguity.new(self, from, to, associated_candidates)
            end

            if type_candidates.size == 1
                return type_candidates.first
            elsif type_candidates.size > 1
                raise TransformationPortAmbiguity.new(self, from, to, type_candidates)
            end

            return nil
        end

        def find_all_input_ports_for_transform(from, to)
            each_transform_input.map do |port, port_from, port_to|
                if port_from == from && port_to == to
                    port
                end
            end.compact
        end

        def each_transform_input
            return enum_for(:each_transform_input) if !block_given?
            each_input_port do |port|
                next if !Transformer.transform_port?(port)

                self.each_concrete_input_connection(port) do |source_task, source_port, sink_port, policy|
                    return if !(tr = source_task.model.transformer)
                    if associated_transform = tr.find_transform_of_port(source_port)
                        from = source_task.selected_frames[associated_transform.from]
                        to   = source_task.selected_frames[associated_transform.to]
                        yield(port, from, to)
                    end
                end
            end
        end

        def each_transform_output
            return enum_for(:each_transform_output) if !block_given?
            each_output_port do |port|
                next if !Transformer.transform_port?(port)

                self.each_concrete_input_connection(port) do |source_task, source_port, sink_port, policy|
                    return if !(tr = source_task.model.transformer)
                    if associated_transform = tr.find_transform_of_port(source_port)
                        from = source_task.selected_frames[associated_transform.from]
                        to   = source_task.selected_frames[associated_transform.to]
                        yield(port, from, to)
                    end
                end
            end
        end

        def select_port_for_transform(port, from, to)
            each_concrete_input_connection(port) do |source_task, source_port, sink_port, policy|
                source_task.select_port_for_transform(source_task.find_port(source_port), from, to)
                return
            end
	    nil
        end

        def find_transform_of_port(port)
            self.each_concrete_input_connection(port) do |source_task, source_port, sink_port, policy|
                return source_task.find_transform_of_port(source_task.find_port(source_port))
            end
	    nil
        end
    end
end

