require 'syskit/test/self'
require 'transformer/syskit'

module Transformer
    module SyskitPlugin
        module SelfTest
            def setup
                super
                Orocos.export_types = true
                Syskit.conf.transformer_warn_about_unset_frames = false
                Syskit.conf.transformer_enabled = true
            end
        end
        Minitest::Test.include SelfTest
    end
end


