require 'sdf'

module Transformer
    module SDF
        # Load a SDF model or file and convert it into a transformer configuration
        def load_sdf(model, &producer_resolver)
            if model.respond_to?(:to_str)
                model = ::SDF::Root.load(model)
            end
            parse_sdf(model, [], &producer_resolver)
        end

        # @api private
        def parse_sdf(sdf, prefix, &producer_resolver)
            if sdf.respond_to?(:each_world)
                sdf.each_world do |w|
                    frames w.full_name
                    parse_sdf(w, prefix + [w.name], &producer_resolver)
                end
            end
            if sdf.respond_to?(:each_model)
                sdf.each_model do |m|
                    frames m.full_name

                    if m.parent.name
                        if m.static?
                            static_transform(*m.pose, m.full_name => m.parent.full_name)
                        else
                            example_transform(*m.pose, m.full_name => m.parent.full_name)
                        end
                    end

                    parse_sdf(m, prefix + [m.name], &producer_resolver)
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

                    parent2model = parent.pose
                    child2model  = child.pose
                    joint2child  = j.pose
                    child2parent = parent2model.inverse * child2model
                    joint2parent = child2parent * joint2child

                    parent = parent.full_name
                    child  = child.full_name
                    joint_pre  = "#{j.full_name}_pre"
                    joint_post = "#{j.full_name}_post"
                    register_joint(joint_post, joint_pre, j)
                    static_transform(joint2child, joint_post => child)
                    static_transform(joint2parent, joint_pre => parent)
                    if producer_resolver && (p = producer_resolver.call(j))
                        dynamic_transform p, joint_post => joint_pre
                    end

                    axis_limit = j.axis.limit
                    upper = axis_limit.upper || 0
                    lower = axis_limit.lower || 0
                    axis = j.axis.xyz
                    if j.axis.use_parent_model_frame?
                        # The axis is expressed in the parent model frame ...
                        # Convert to joint frame
                        joint2model = (joint2child * child2model)
                        axis = joint2model.rotation.inverse * axis
                    end
                    post2pre = j.transform_for((upper + lower) / 2, axis)
                    example_transform post2pre, joint_post => joint_pre
                end

                root_links.each_value do |l|
                    static_transform(*l.pose, l.full_name => l.parent.full_name)
                end
            end
        end
    end
    Transformer::Configuration.include SDF
end
