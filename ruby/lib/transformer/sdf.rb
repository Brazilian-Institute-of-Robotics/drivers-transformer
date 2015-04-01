require 'sdf'

module Transformer
    module SDF
        # Load a SDF model or file and convert it into a transformer configuration
        def self.load(model_name_or_path, &producer_resolver)
            sdf = ::SDF::Root.load(model_name_or_path)
            load_from_sdf(sdf, &producer_resolver)
        end

        # Create a transformer configuration from SDF model
        #
        # @param [SDF::Element] the root of the SDF data that should be used
        # @return [Transformer::Configuration]
        def self.load_from_sdf(sdf, &producer_resolver)
            result = Transformer::Configuration.new
            parse_sdf(result, sdf, [], &producer_resolver)
            result
        end

        # @api private
        def self.parse_sdf(conf, sdf, prefix, &producer_resolver)
            if sdf.respond_to?(:each_world)
                sdf.each_world do |w|
                    conf.frames w.full_name
                    parse_sdf(conf, w, prefix + [w.name], &producer_resolver)
                end
            end
            if sdf.respond_to?(:each_model)
                sdf.each_model do |m|
                    conf.frames m.full_name

                    if m.parent.name
                        if m.static?
                            conf.static_transform(*m.pose, m.full_name => m.parent.full_name)
                        else
                            conf.example_transform(*m.pose, m.full_name => m.parent.full_name)
                        end
                    end

                    parse_sdf(conf, m, prefix + [m.name], &producer_resolver)
                end
            end
            if sdf.respond_to?(:each_link)
                root_links = Hash.new
                sdf.each_link do |l|
                    root_links[l.name] = l
                end

                sdf.each_joint do |j|
                    parent = j.parent_link
                    child  = j.child_link
                    root_links.delete(child.name)

                    parent = parent.full_name
                    child  = child.full_name
                    if producer_resolver && (p = producer_resolver.call(parent, child))
                        conf.dynamic_transform p, child => parent
                    end

                    axis = j.axis
                    axis_v = axis.xyz
                    upper = axis.limit.upper || 0
                    lower = axis.limit.lower || 0
                    if j.type == 'revolute'
                        v, q = Eigen::Vector3.Zero, Eigen::Quaternion.from_angle_axis((upper + lower) / 2, axis_v)
                    else
                        v, q = xyz.normalize * (upper + lower) / 2, Eigen::Quaternion.Identity
                    end
                    conf.example_transform v, q, child => parent
                end

                root_links.each_value do |l|
                    conf.static_transform(*l.pose, l.full_name => l.parent.full_name)
                end
            end
        end
    end
end
