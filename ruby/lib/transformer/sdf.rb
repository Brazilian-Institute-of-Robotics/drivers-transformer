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

                    parent = parent.full_name
                    child  = child.full_name
                    joint = j.full_name
                    static_transform(*j.pose, joint => child)
                    if producer_resolver && (p = producer_resolver.call(parent, joint))
                        dynamic_transform p, joint => parent
                    end

                    axis_limit = j.axis.limit
                    upper = axis_limit.upper || 0
                    lower = axis_limit.lower || 0
                    v, q = j.transform_for((upper + lower) / 2)
                    example_transform v, q, joint => parent
                end

                root_links.each_value do |l|
                    static_transform(*l.pose, l.full_name => l.parent.full_name)
                end
            end
        end
    end
    Transformer::Configuration.include SDF
end
