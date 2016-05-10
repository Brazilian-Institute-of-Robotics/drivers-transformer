module Transformer
    module ProfileRobotDefinitionExtension
        def inject_di_context(requirements)
            requirements.transformer.merge(profile.transformer)
            super
        end
    end
end
