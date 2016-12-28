require 'sdf'

module Transformer
    module SDF
        # Load a SDF model or file and convert it into a transformer configuration
        def load_sdf(model, exclude_models: [], &producer_resolver)
            model = ::SDF::Root.load(model)
            parse_sdf_root(model, exclude_models: exclude_models, &producer_resolver)
        end

        def parse_sdf_root(sdf, exclude_models: [], &producer_resolver)
            parse_sdf(sdf, "", "", exclude_models: exclude_models, &producer_resolver)
        end

        def parse_sdf_world(sdf, exclude_models: [], &producer_resolver)
            frames sdf.full_name
            parse_sdf(sdf, "", sdf.full_name, exclude_models: exclude_models, &producer_resolver)
        end

        def sdf_append_name(prefix, name)
            if prefix.empty? then name
            else "#{prefix}::#{name}"
            end
        end

        def parse_sdf_model(sdf, prefix = "", parent_name = "", exclude_models: [], &producer_resolver)
            full_name = sdf_append_name(prefix, sdf.name)
            frames full_name

            if !parent_name.empty?
                if sdf.static?
                    static_transform(*sdf.pose, sdf.name => parent_name)
                else
                    example_transform(*sdf.pose, sdf.name => parent_name)
                end
            end

            parse_sdf(sdf, full_name, full_name, exclude_models: exclude_models, &producer_resolver)
        end

        def parse_sdf_links_and_joints(sdf, prefix = "", parent_name = "", &producer_resolver)
            root_links = Hash.new
            sdf.each_link do |l|
                root_links[l.name] = l
            end

            sdf.each_joint do |j|
                parent = j.parent_link
                child  = j.child_link
                root_links.delete(child.name)

                parent2model = parent.pose
                child2model  = child.pose
                joint2child  = j.pose
                child2parent = parent2model.inverse * child2model
                joint2parent = child2parent * joint2child

                if j.type != 'fixed'
                    axis_limit = j.axis.limit
                    upper = axis_limit.upper || 0
                    lower = axis_limit.lower || 0
                    axis = j.axis.xyz
                    if j.axis.use_parent_model_frame?
                        # The axis is expressed in the parent model frame ...
                        # Convert to joint frame
                        joint2model = child2model * joint2child
                        axis = joint2model.rotation.inverse * axis
                    end
                    post2pre = j.transform_for((upper + lower) / 2, axis)
                end

                parent = sdf_append_name(parent_name, parent.name)
                child  = sdf_append_name(parent_name, child.name)
                if upper == lower
                    static_transform child2parent, child => parent
                else
                    joint_pre  = sdf_append_name(parent_name, "#{j.name}_pre")
                    joint_post = sdf_append_name(parent_name, "#{j.name}_post")
                    register_joint(joint_post, joint_pre, j)
                    static_transform(joint2child, joint_post => child)
                    static_transform(joint2parent, joint_pre => parent)
                    if producer_resolver && (p = producer_resolver.call(j))
                        dynamic_transform p, joint_post => joint_pre
                    end
                    example_transform post2pre, joint_post => joint_pre
                end
            end

            root_links.each_value do |l|
                static_transform(*l.pose, sdf_append_name(prefix, l.name) => parent_name)
            end
        end

        # @api private
        def parse_sdf(sdf, prefix, parent_name, exclude_models: [], &producer_resolver)
            if sdf.respond_to?(:each_world)
                sdf.each_world { |w| parse_sdf_world(w, exclude_models: exclude_models, &producer_resolver) }
            end
            if sdf.respond_to?(:each_model)
                sdf.each_model do |m|
                    next if exclude_models.include?(m.name)
                    parse_sdf_model(m, prefix, parent_name, exclude_models: exclude_models, &producer_resolver)
                end
            end
            if sdf.respond_to?(:each_link)
                parse_sdf_links_and_joints(sdf, prefix, parent_name, &producer_resolver)
            end
        end
    end
    Transformer::Configuration.include SDF
end
