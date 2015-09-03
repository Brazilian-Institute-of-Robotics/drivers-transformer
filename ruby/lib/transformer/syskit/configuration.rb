module Transformer
    class SyskitConfiguration < Configuration
        def dynamic_transform(producer, *frames)
            producer = producer.to_instance_requirements
            super(producer, *frames)
        end
    end
end

