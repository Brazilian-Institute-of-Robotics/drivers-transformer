require 'utilrb/kernel/load_dsl_file'
require 'eigen'
require 'set'
require 'utilrb/logger'

# Ruby-side library for Transformer functionality
#
# The Ruby-side library can be used to declare the transformation production
# graph (i.e. who is producing which transformations), and compute
# transformation chains between two points in this graph
module Transformer
    extend Logger::Root("Transformer", Logger::WARN)

    # True if +port+ is a valid port to hold transformation information
    def self.transform_port?(port)
        transform_type?(port.type)
    end

    # True if +type+ is a type to represent transformations
    #
    # @arg [String,Typelib::Type] the typename or type object to test against
    def self.transform_type?(type)
        if type.respond_to?(:name)
            type = type.name
        end
        return type == "/base/samples/RigidBodyState"
    end

    # A geometric frame, referenced to by name
    class Frame
        # The name of the frame
        attr_accessor :name    

        def hash; name.hash end
        def eql?(other)
            other.kind_of?(Frame) &&
                other.name == name
        end
        def ==(other)
            other.kind_of?(Frame) &&
                other.name == name
        end
    end

    # Representation of a frame transformation
    #
    # The frames are represented by their name
    class Transform
        # Name of the source frame
        attr_reader :from
        # Name of the target frame
        attr_reader :to   

        def initialize(from, to)
            @from = from
            @to = to
        end

        def pretty_print(pp)
            pp.text "#{from}2#{to}"
        end
    end

    # Represents a frame transformation that has static value
    class StaticTransform < Transform
        attr_accessor :translation
        attr_accessor :rotation

        def initialize(from, to, translation, rotation)
            super(from, to)
            @translation, @rotation = translation, rotation
        end

        def pretty_print(pp)
            super
            pp.text ": static"
        end
    end

    # Represents a frame transformation that is generated dynamically
    #
    # The producer object must respond to #inverse
    class DynamicTransform < Transform
        attr_reader :producer

        def initialize(from, to, producer)
            super(from, to)
            @producer = producer
        end
        
        def pretty_print(pp)
            super
            pp.text ": produced by #{producer}"
        end
    end

    # A transformation that is produced by a chain of transformations
    #
    # The constraint is that, when the links in +links+ are chained together,
    # they form a chain from +self.from+ to +self.to+
    class TransformChain < Transform
        # Array of Transform objects representing the elements of the chain
        attr_reader :links
        # Array of the same size than +links+. If an element at index +idx+ is
        # true, then the link at +links[idx]+ must be inversed before it gets
        # concatenated
        attr_reader :inversions

        # Initializes the object using a leaf TransformNode
        #
        # The object will be initialized by traversing the parents of
        # +transformation_node+.
        def initialize(transformation_node)
            @links = Array.new
            @inversions = Array.new

            to = transformation_node.frame
            cur_node = transformation_node
            while cur_node.parent
                @links.unshift(cur_node.link_to_parent)
                @inversions.unshift(cur_node.inverse)
                cur_node = cur_node.parent
            end

            super(cur_node.frame, to)
        end

        # Returns the set of static transformations and producers needed to
        # form this chain
        def partition
            @links.partition do |link|
                link.kind_of?(StaticTransform)
            end
        end

        def pretty_print(pp)
            pp.text "Transform Chain: #{from} to #{to} "
            pp.breakable
            pp.text "Links:"
            pp.nest(2) do
                pp.breakable
                pp.seplist(links.each_with_index) do |tr|
                    tr, i = *tr
                    pp.text("(inv)") if inversions[i]
                    tr.pretty_print(pp)
                end
            end
        end
    end

    # A node used during discovery to find transformation chains
    #
    # A TransformationNode is an element in a list, containing a back-pointer to
    # its parent in the chain. It is only used during transformation chain
    # discovery to represent the possible paths.
    class TransformNode
        # The frame of this node, as a frame name
        attr_reader :frame
        # The parent TransformNode object
        attr_reader :parent
        # The Transform object that links +from+ to +frame+. If +inverse+ is
        # false, it is a transformation from +from.frame+ to +self.frame+. If
        # +inverse+ is true, it is the opposite transformation.
        attr_reader :link_to_parent
        # Flag representing if +link_to_parent+ should be concatenated in the
        # chain as-is, or inverted first
        attr_reader :inverse
        # The complete path, as a list of [from_frame, to_frame] pairs. Both
        # +from_frame+ and +to_frame+ are frame names. It is used to detect
        # cycles in the discovery process and avoid them
        attr_reader :traversed_links

        def initialize(frame, parent, link_to_parent, inverse)
            @parent = parent
            @frame = frame
            @link_to_parent = link_to_parent
            @inverse = inverse

            @traversed_links =
                if parent
                    @traversed_links = parent.traversed_links.dup
                    @traversed_links << [frame, parent.frame].to_set
                else []
                end
        end
    end

    # Transformer algorithm
    #
    # This class contains the complete transformation configuration, and can
    # return transformation chains between two frames in the configuration.
    #
    # It requires two objects:
    #
    # * a Configuration object that contains the set of frames, static and
    #   dynamic transformations
    # * a ConfigurationChecker object that can validate the various parts in the
    #   configuration
    #
    class TransformationManager
        # The object that holds frame and transformation definitions. It is
        # usually a Configuration object
        attr_accessor :conf
        # The object that validates the contents in +configuration+. It is
        # usually a ConfigurationChecker object, or a subclass of it.
        attr_accessor :checker

        # In order to find transformation chains, the transformer performs a
        # graph search. This is the maximum depth of that search
        attr_reader :max_seek_depth

        def initialize(max_seek_depth = 50, &producer_check)
            @max_seek_depth = max_seek_depth;
            @checker = ConfigurationChecker.new(producer_check)
            @conf = Configuration.new(@checker)
        end

        # Loads a configuration file. See the documentation of Transformer for
        # the syntax
        #
        # If multiple arguments are provided, they are joined with File.join
        def load_configuration(*config_file)
            eval_dsl_file(File.join(*config_file), @conf, [], false)
        end

        # Returns the set of transformations in +transforms+ where
        #
        # * +node+ is a starting point 
        # * the transformation is not +node.parent+ => +node+
        #
        # The returned array is an array of elements [transformation, inverse]
        # where +transformation+ is an instance of a subclass of Transform,
        # and +inverse+ is true if +Transform+ should be taken in a reverse
        # way and false otherwise
        def matching_transforms(node, transforms)
            ret = Array.new

            transforms[node.frame].each do |i|
                link_marker = [i.from, i.to].to_set
                if !node.traversed_links.include?(link_marker)
                    ret << [i, false]
                    ret << [i, true]
                end
            end

            return ret
        end

        # Returns the shortest transformation chains that link +from+ to +to+
        def transformation_chain(from, to, additional_producers = Hash.new)
            from = from.to_s
            to = to.to_s
            checker.check_frame(from, conf.frames)
            checker.check_frame(to, conf.frames)

            known_transforms = Set.new
            all_transforms = Hash.new { |h, k| h[k] = Set.new }
            additional_producers.each do |(add_from, add_to), producer_name|
                trsf = DynamicTransform.new(add_from, add_to, producer_name)
                all_transforms[trsf.from] << trsf
                all_transforms[trsf.to]   << trsf
                known_transforms << [trsf.from, trsf.to] << [trsf.to, trsf.from]
            end

            conf.transforms.each_value do |trsf|
                if !known_transforms.include?([trsf.from, trsf.to])
                    all_transforms[trsf.from] << trsf
                    all_transforms[trsf.to]   << trsf
                    known_transforms << [trsf.from, trsf.to] << [trsf.to, trsf.from]
                end
            end

            possible_next_nodes, next_level = Array.new, Array.new
            possible_next_nodes.push(TransformNode.new(from, nil, nil, false))

            max_depth = [@max_seek_depth, known_transforms.size / 2].min
            max_depth.times do
                # Iterate over the possible next nodes, and add them to all
                # existing chains
                possible_next_nodes.each do |node|
                    links_for_node = matching_transforms(node, all_transforms)
                    links_for_node.each do |link, inverse|
                        target_frame =
                            if inverse then link.from
                            else link.to
                            end

                        child_node = TransformNode.new(target_frame, node, link, inverse)
                        if target_frame == to
                            return TransformChain.new(child_node)
                        end
                        next_level << child_node
                    end
                end

                if next_level.empty?
                    raise ArgumentError, "no transformation from '#{from}' to '#{to}' available"
                end

                possible_next_nodes, next_level = next_level, possible_next_nodes
                next_level.clear
            end
            raise ArgumentError, "max seek depth reached seeking Transform from '#{from}' to '#{to}'"
        end
    end

    class InvalidConfiguration < RuntimeError; end

    # This class is used to validate the transformer configuration, as well as
    # parameters given to the transformer calls
    class ConfigurationChecker
        attr_accessor :producer_check

        def initialize(producer_check = nil)
            @producer_check = producer_check || lambda {}
        end

        def check_transformation_frames(frames, transforms)
            transforms.each do |i|
                check_transformation(frames, i)
            end
        end

        def check_transformation(frames, transformation)
            errors = []
            if(!frames.include?(transformation.from))
                errors << "transformation from #{transformation.from} to #{transformation.to} uses unknown frame #{transformation.from}, known frames: #{frames.to_a.sort.join(", ")}"
            end	

            if(!frames.include?(transformation.to))
                errors << "transformation from #{transformation.from} to #{transformation.to} uses unknown frame #{transformation.to}, known frames: #{frames.to_a.sort.join(", ")}"
            end
            if !errors.empty?
                raise InvalidConfiguration, "transformation configuration contains errors:\n  " + errors.join("\n  ")
            end
        end

        def check_frame(frame, frames = nil)
            frame = frame.to_s
            if frame !~ /^\w+$/
                raise InvalidConfiguration, "frame names can only contain alphanumeric characters and _, got #{frame}"
            end

            if(frames && !frames.include?(frame))
                raise InvalidConfiguration, "unknown frame #{frame}"
            end
        end

        def check_producer(producer)
            @producer_check.call(producer)
        end
    end

    # Class that represents the transformer configuration
    class Configuration
        attr_accessor :transforms
        attr_accessor :frames
        attr_accessor :checker

        def initialize(checker = ConfigurationChecker.new)
            @transforms = Hash.new
            @frames = Set.new
            @checker = checker
        end

        # Declares frames
        #
        # Frames need to be declared before they are used in the
        # #static_transform and #dynamic_transform calls
        def frames(*frames)
            frames.map!(&:to_s)
            frames.each do |i|
                checker.check_frame(i)
            end
            @frames |= frames.to_set
        end

        # True if +frame+ is a defined frame
        def has_frame?(frame)
            self.frames.include?(frame.to_s)
        end

        def parse_transform_hash(hash, expected_size)
            if expected_size && hash.size != expected_size
                raise ArgumentError, "expected #{expected_size} transformation(s), got #{hash}"
            end

            hash.to_a
        end
        def parse_single_transform(hash)
            return parse_transform_hash(hash, 1).first
        end

        # call-seq:
        #   dynamic_transform "from_frame", "to_frame", producer
        #
        # Declares a new dynamic transformation. Acceptable values for
        # +producer+ depend on the currently selected checker (i.e. on the
        # current use-case)
        #
        # For instance, producers in orocos.rb are strings that give the name of
        # the task context that will provide that transformation
        def dynamic_transform(producer, transform)
            from, to = parse_single_transform(transform)
            frames(from, to)

            checker.check_producer(producer)
            tr = DynamicTransform.new(from, to, producer)
            checker.check_transformation(frames, tr)
            transforms[[from, to]] = tr
        end

        # call-seq:
        #   static_transform "from_frame", "to_frame", translation
        #   static_transform "from_frame", "to_frame", rotation
        #   static_transform "from_frame", "to_frame", translation, rotation
        #
        # Declares a new static transformation
        def static_transform(*transformation)
            from, to = parse_single_transform(transformation.pop)
            frames(from, to)

            if transformation.empty?
                raise ArgumentError, "no transformation given"
            elsif transformation.size <= 2
                translation, rotation = transformation
                if translation.kind_of?(Eigen::Quaternion)
                    translation, rotation = rotation, translation
                end
                translation ||= Eigen::Vector3.new(0, 0, 0)
                rotation    ||= Eigen::Quaternion.Identity

                if !translation.kind_of?(Eigen::Vector3)
                    raise ArgumentError, "the provided translation is not an Eigen::Vector3"
                end
                if !rotation.kind_of?(Eigen::Quaternion)
                    raise ArgumentError, "the provided rotation is not an Eigen::Quaternion"
                end
            else
                raise ArgumentError, "#static_transform was expecting either a translation, rotation or both but got #{transformation}"
            end

            tr = StaticTransform.new(from, to, translation, rotation)
            checker.check_transformation(frames, tr)
            transforms[[from, to]] = tr
        end

        # Checks if a transformation between the provided frames exist.
        #
        # It will return true if such a transformation has been registered,
        # false otherwise, and raises ArgumentError if either +from+ or +to+ are
        # not registered frames.
        def has_transformation?(from, to)
            result = transforms.has_key?([from, to])

            if !result
                if !has_frame?(from)
                    raise ArgumentError, "#{from} is not a registered frame"
                elsif !has_frame?(to)
                    raise ArgumentError, "#{to} is not a registered frame"
                end
            end
            result
        end

        # Returns the transformation object that represents the from -> to
        # transformation, if there is one. If none is found, raises
        # ArgumentError
        def transformation_for(from, to)
            result = transforms[[from, to]]
            if !result
                if !has_frame?(from)
                    raise ArgumentError, "#{from} is not a registered frame"
                elsif !has_frame?(to)
                    raise ArgumentError, "#{to} is not a registered frame"
                else
                    raise ArgumentError, "there is no registered transformations between #{from} and #{to}"
                end
            end
            result
        end

        def each_static_transform
            transforms.each_value do |val|
                yield(val) if val.kind_of?(StaticTransform)
            end
        end

        def each_dynamic_transform
            transforms.each_value do |val|
                yield(val) if val.kind_of?(DynamicTransform)
            end
        end


        def pretty_print(pp)
            pp.test "Transformer configuration"
            pp.nest(2) do
                pp.breakable
                pp.text "Available Frames:"
                pp.nest(2) do
                    frames.each do |i| 
                        pp.breakable
                        i.pretty_print(pp)
                    end
                end

                pp.breakable
                pp.text "Static Transforms:"
                pp.nest(2) do
                    each_static_transform do |i|
                        pp.breakable
                        i.pretty_print(pp)
                    end
                end

                pp.breakable
                pp.text "Dynamic Transforms:"
                pp.nest(2) do
                    each_dynamic_transform do |i|
                        pp.breakable
                        i.pretty_print(pp)
                    end
                end
            end
        end

    end
end

