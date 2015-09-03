module Transformer
    module PlanExtension
        attribute(:transformer_configuration_state) do
            [Time.now, Types.transformer.ConfigurationState.new]
        end

        # Propagates the transformer_configuration_state changes to the underlying plan
        def committed_transaction
            super if defined? super
            plan.transformer_configuration_state[0] = transformer_configuration_state[0]
            plan.transformer_configuration_state[1] = transformer_configuration_state[1].dup
        end
    end
end


