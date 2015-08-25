module Transformer
    # Module mixed-in in component models that are represented as orogen
    # interfaecs
    module ConcreteComponentExtension
        # Returns an output port object that is providing the requested
        # transformation, or nil if none can be found
        #
        # Raises TransformationPortAmbiguity if multiple ports match.
        def find_port_for_transform(from, to)
            not_candidates = []
            candidates = []
            each_transform_output do |port, port_from, port_to|
                if port_from == from && port_to == to
                    return port
                elsif ((!port_from || port_from == from) && (!port_to || port_to == to))
                    candidates << port
                else
                    not_candidates << port
                end
            end

            if candidates.size == 1
                return candidates.first
            elsif candidates.size > 1
                raise TransformationPortAmbiguity.new(self, from, to, candidates)
            end

            model.each_output_port do |port|
                next if not_candidates.include?(port)
                if Transformer.transform_port?(port)
                    candidates << port.bind(self)
                end
            end

            if candidates.size == 1
                return candidates.first
            elsif candidates.size > 1
                raise TransformationPortAmbiguity.new(self, from, to, candidates)
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
    end
end
