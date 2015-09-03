module Transformer
    module ConfigurationExtension
        attr_predicate :transformer_enabled?

        def transformer_enabled=(value)
            if value
                Transformer::SyskitPlugin.enable
            end
            @transformer_enabled = value
        end

        def transformer_warn_about_unset_frames?
            if !instance_variable_defined?(:@transformer_warn_about_unset_frames)
                true
            else @transformer_warn_about_unset_frames
            end
        end

        def transformer_warn_about_unset_frames=(flag)
            @transformer_warn_about_unset_frames = flag
        end

	def transformer_broadcaster_enabled?
	    transformer_enabled? && (@transformer_broadcaster_enabled != false)
	end

        def transformer_broadcaster_enabled=(value)
	    @transformer_broadcaster_enabled = value
        end
    end
end

