module Transformer
    # Extension to syskit profiles
    #
    # It gives access to the transformer configuration
    module ProfileExtension
        # This is a normal transformer configuration with only a few added
        # tweaks
        class Configuration < SyskitConfiguration
            attr_accessor :profile
            def initialize(profile)
                @profile = profile
                super()
            end

            def load(*path)
                args = path.dup
                args << Hash[:order => :specific_first]
                if !Roby.app.find_file(*args)
                    raise ArgumentError, "cannot find #{File.join(*path)}"
                end
                super(path)
            end

            def method_missing(m, *args, &block)
                if m.to_s =~ /_dev$/
                    profile.send(m, *args, &block)
                elsif m.to_s =~ /(.*)_def$/
                    profile.resolved_definition($1, *args, &block)
                else super
                end
            end
        end

        def transformer
            if !@transformer
                @transformer = Configuration.new(self)
                all_used_profiles.each do |profile, tags|
                    if profile.has_transformer?
                        use_profile_transformer(profile, tags)
                    end
                end
            end
            if block_given?
                @transformer.instance_eval(&proc)
            end
            @transformer
        end

        def has_transformer?
            !!@transformer
        end

        def use_profile_transformer(profile, tags = Hash.new)
            tr = profile.transformer.dup
            tr.each_dynamic_transform do |dyn|
                dyn.producer = promote_requirements(profile, dyn.producer.to_instance_requirements, tags)
            end
            tr.profile = self
            if has_transformer?
                tr.merge(@transformer)
            end
            @transformer = tr
        end

        def use_profile(profile, tags = Hash.new)
            super if defined? super
            if profile.has_transformer?
                use_profile_transformer(profile, tags)
            end
        end

        # Called by Profile to modify an instance requirement object w.r.t. the
        # transformer information 
        def inject_di_context(req)
            super if defined? super
            req.transformer.merge(transformer)
        end

        def clear_model
            super if defined? super
            @transformer = nil
        end
    end
end
