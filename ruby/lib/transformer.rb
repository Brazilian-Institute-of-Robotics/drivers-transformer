require 'utilrb/kernel/load_dsl_file'
require 'utilrb/hash/map_value'
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
        return type == "/base/samples/RigidBodyState" || type == "/base/samples/RigidBodyState_m"
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

        def initialize_copy(old)
            super
            @translation = old.translation.dup
            @rotation = old.rotation.dup
        end

        def pretty_print(pp)
            super
            pp.text ": static (xyz=#{translation.to_a.map(&:to_s).join(", ")} rpy=#{rotation.to_euler.to_a.map(&:to_s).join(", ")})"
        end
    end

    # Represents a frame transformation that is generated dynamically
    #
    # The producer object must respond to #inverse
    class DynamicTransform < Transform
        attr_accessor :producer

        def initialize(from, to, producer)
            super(from, to)
            @producer = producer
        end

        def initialize_copy(old)
            super
            @producer = old.producer.dup
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

        # Initializes the chain
        #
        # @overload initialize("frame_name")
        #   @param [String] initial
        #   Sets the chain to an identity transformation between the given frame
        #   name and itself
        # @overload initialize(node)
        #   @param [FrameNode] initial
        #   Initialize the chain by traversing the parents of the initial frame
        #   node. This initial node is basically treated as the leaf node in a
        #   chain
        def initialize(initial)
            @links = Array.new
            @inversions = Array.new

            if initial.respond_to?(:to_str)
                super(initial, initial)
            else
                to = initial.frame
                cur_node = initial
                while cur_node.parent
                    @links.unshift(cur_node.link_to_parent)
                    @inversions.unshift(cur_node.inverse)
                    cur_node = cur_node.parent
                end
                super(cur_node.frame, to)
            end
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
                pp.seplist(links.each_with_index) do |tr, i|
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

	    if parent
		@traversed_links = parent.traversed_links.dup
		if !frame || frame.empty? || !parent.frame || parent.frame.empty?
		    raise ArgumentError, "transformation without a source or target frame"
		elsif frame == parent.frame
		    raise ArgumentError, "trying to create a link from a frame to itself"
		end
		@traversed_links << [frame, parent.frame].to_set
	    else
		@traversed_links = []
	    end
        end
    end
    
    # Exception raised when a transformation requested in #transformation_chain
    # cannot be found
    class TransformationNotFound < RuntimeError
	attr_reader :from
	attr_reader :to

	def initialize(from, to)
	    @from, @to = from, to
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

        def initialize(conf = Configuration.new, max_seek_depth = 50, &producer_check)
            @max_seek_depth = max_seek_depth;
            @conf = conf
            @checker = ConfigurationChecker.new(producer_check)
            conf.checker = @checker
        end

        # Loads a configuration file. See the documentation of Transformer for
        # the syntax
        #
        # If multiple arguments are provided, they are joined with File.join
        def load_configuration(*config_file)
            conf.load(*config_file)
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

            transforms[node.frame].each do |link, inverse|
                link_marker = [link.from, link.to].to_set
                if !node.traversed_links.include?(link_marker)
                    ret << [link, inverse]
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

            if from == to
                return TransformChain.new(from)
            end

            known_transforms = Set.new
            all_transforms = Hash.new { |h, k| h[k] = Set.new }
            additional_producers.each do |(add_from, add_to), producer_name|
		if !add_from
		    raise ArgumentError, "explicitly provided #{producer_name} as a transform producer from a nil frame"
                elsif !add_to
		    raise ArgumentError, "explicitly provided #{producer_name} as a transform producer to a nil frame"
                elsif add_from == add_to
		    raise ArgumentError, "explicitly provided #{producer_name} as a transform producer for #{add_from} onto itself"
		end
                trsf = DynamicTransform.new(add_from, add_to, producer_name)
                all_transforms[trsf.from] << [trsf, false]
                all_transforms[trsf.to]   << [trsf, true]
                known_transforms << [trsf.from, trsf.to] << [trsf.to, trsf.from]
            end

            conf.transforms.each_value do |trsf|
                if !known_transforms.include?([trsf.from, trsf.to])
                    all_transforms[trsf.from] << [trsf, false]
                    all_transforms[trsf.to]   << [trsf, true]
                    known_transforms << [trsf.from, trsf.to] << [trsf.to, trsf.from]
                end
            end

            possible_next_nodes, next_level = Array.new, Array.new
            possible_next_nodes.push(TransformNode.new(from, nil, nil, false))

            max_depth = [@max_seek_depth, known_transforms.size * 2 + 1].min
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
                    raise TransformationNotFound.new(from, to), "no transformation from '#{from}' to '#{to}' available"
                end

                possible_next_nodes, next_level = next_level, possible_next_nodes
                next_level.clear
            end
            raise TransformationNotFound.new(from, to), "max seek depth reached seeking Transform from '#{from}' to '#{to}'"
        end
    end

    class InvalidConfiguration < RuntimeError; end

    # This class is used to validate the transformer configuration, as well as
    # parameters given to the transformer calls
    class ConfigurationChecker
        attr_accessor :producer_check

        def initialize(producer_check = nil)
            @producer_check = producer_check || lambda { |_| }
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
            if(frames && !frames.include?(frame))
                raise InvalidConfiguration, "unknown frame #{frame}, known frames: #{frames.to_a.sort.join(", ")}"
            end
        end

        def check_producer(producer)
            if !producer
                raise ArgumentError, "cannot have a nil producer"
            end
            @producer_check.call(producer)
        end
    end

    # Class that represents the transformer configuration
    class Configuration
        # The set of known transformations
        #
        # @return [Hash<(String,String),StaticTransform|DynamicTransform>]
        attr_accessor :transforms

        # The set of registered example transformations
        #
        # @return [Hash<(String,String),StaticTransform>]
        attr_accessor :example_transforms

        # The set of known frame names
        #
        # @return [Set<String>]
        attr_accessor :frames
        attr_accessor :checker

        # A set of joints that are part of this configuration
        #
        # This is filled by side-loaders to export it to use, but it is not used
        # by the transformer code itself
        #
        # @return [(String,String,Object)]
        attr_reader :joints

        def initialize(checker = ConfigurationChecker.new)
            @transforms = Hash.new
            @frames = Set.new
            @example_transforms = Hash.new
            @checker = checker
            @joints = Array.new
        end

        def initialize_copy(old)
            @transforms = Hash.new
            @frames = Set.new
            @example_transforms = Hash.new
            @checker = old.checker
            @joints = Array.new
            merge(old)
        end

        # Returns true if this transformer configuration and the given one are
        # compatible, i.e. if #merge would not remove any information
        def compatible_with?(other)
            transforms.each do |fromto, tr|
                next if !other.transforms.has_key?(fromto)
                return false if other.transforms[fromto] != tr
            end
            true
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

        # Load a transformer configuration file
        def load(*conf_file)
	    begin
		file_name = File.join(*conf_file)
	    rescue TypeError => e
		raise ArgumentError, "could not create path object from #{conf_file}"
	    end
	    Transformer.info "loading configuration file #{File.join(*conf_file)}"
            eval_dsl_file(File.join(*conf_file), self, [], false)
        end

        # Used in the transformer configuration files to load other files. This
        # is only an alias to #load
        def load_transformer_conf(*conf_file)
            load(*conf_file)
        end

        # True if +frame+ is a defined frame
        def has_frame?(frame)
            self.frames.include?(frame.to_s)
        end

        def empty?
            transforms.empty?
        end

        # Adds the information from another Configuration object to self.
        # In case some definitions are colliding, the information from +conf+ is
        # used
        #
        # @param [Configuration] conf
        # @return self
        def merge(conf)
            transforms.merge!(conf.transforms.map_value { |_, v| v.dup })
            example_transforms.merge!(conf.example_transforms.map_value { |_, v| v.dup })
            @frames |= conf.frames
            joints |= conf.joints
            self
        end

        def clear
            transforms.clear
            example_transforms.clear
            frames.clear
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
        #   dynamic_transform producer, "from_frame" => "to_frame"
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
	    add_transform(tr)
        end
    
        # Declare a transformation
        #
        # @see static_transform dynamic_transform
	def add_transform(tr)
            checker.check_transformation(frames, tr)
	    if tr.from == tr.to
		raise ArgumentError, "trying to register a transformation from #{tr.from} onto itself"
	    end
            transforms[[tr.from, tr.to]] = tr
	end

        # Registers a transformation object as begin an example transformation
        # for a given frame change
        #
        # @see example_transform
        def add_example_transform(tr)
            checker.check_transformation(frames, tr)
	    if tr.from == tr.to
		raise ArgumentError, "trying to register a transformation from #{tr.from} onto itself"
	    end
            example_transforms[[tr.from, tr.to]] = tr
        end

        # @api private
        #
        # Validates that the arguments passed to {static_transform} and
        # {example_transform} match the required format, and normalizes them
        #
        # @overload validate_static_transform_arguments(position, rotation, from => to)
        # @overload validate_static_transform_arguments(rotation, from => to)
        # @overload validate_static_transform_arguments(position, from => to)
        #
        # @return [Eigen::Vector3,Eigen::Quaternion]
        def validate_static_transform_arguments(*transformation)
            from, to = parse_single_transform(transformation.pop)
            if !from || from.empty?
                raise ArgumentError, "nil or empty frame given for 'from'"
            end
            if !to || to.empty?
                raise ArgumentError, "nil or empty frame given for 'from'"
            end
            frames(from, to)

            if transformation.empty?
                raise ArgumentError, "no transformation given"
            elsif transformation.size > 2
                raise ArgumentError, "was expecting either a translation, rotation or both but got #{transformation}"
            end

            translation, rotation = transformation
            if translation.kind_of?(Eigen::Isometry3)
                translation, rotation = translation.translation, translation.rotation
            elsif translation.kind_of?(Eigen::Quaternion)
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
            return from, to, translation, rotation
        end

        # Declares a new static transformation
        #
        # @overload static_transform translation, 'from_frame' => 'to_frame'
        #   Adds a transformation with the specified translation and an identity
        #   rotation
        #   @param [Eigen::Vector3] translation the translation
        #   @return [StaticTransform]
        #
        # @overload static_transform rotation, 'from_frame' => 'to_frame'
        #   Adds a transformation with the specified rotation and a zero
        #   translation
        #   @param [Eigen::Quaternion] rotation the rotation
        #   @return [StaticTransform]
        #
        # @overload static_transform translation, rotation, 'from_frame' => 'to_frame'
        #   Adds a transformation with the specified translation and rotation
        #
        #   @param [Eigen::Quaternion] rotation the rotation
        #   @return [StaticTransform]
        #
        def static_transform(*transformation)
            from, to, translation, rotation = validate_static_transform_arguments(*transformation)
            tr = StaticTransform.new(from, to, translation, rotation)
	    add_transform(tr)
            tr
        end

        # Declares an example transformation between two frames
        #
        # This is mostly useful for visualization and design tools which can't
        # know what values a dynamic transformation would take. The example
        # transformation can then be used to show a meaningful transform state
        # instead of taking identity each time
        #
        # @see find_example_transform
        # @return [StaticTransform]
        def example_transform(*transformation)
            from, to, translation, rotation = validate_static_transform_arguments(*transformation)
            tr = StaticTransform.new(from, to, translation, rotation)
	    add_example_transform(tr)
            tr
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
                    raise ArgumentError, "#{from} is not a registered frame (#{frames.to_a.sort.join(", ")})"
                elsif !has_frame?(to)
                    raise ArgumentError, "#{to} is not a registered frame (#{frames.to_a.sort.join(", ")})"
                else
                    raise ArgumentError, "there is no registered transformations between #{from} and #{to}"
                end
            end
            result
        end

        # Returns an example transformation for the given set of frames
        #
        # @raise ArgumentError if one of the two frames are not declared
        # @return [StaticTransform] either an example explicitely declared with
        #   {example_transform}, or a StaticTransform object that represents
        #   identity
        def example_transform_for(from, to)
            if result = example_transforms[[from, to]]
                result
            elsif !has_frame?(from)
                raise ArgumentError, "#{from} is not a registered frame"
            elsif !has_frame?(to)
                raise ArgumentError, "#{to} is not a registered frame"
            else
                StaticTransform.new(from, to, Eigen::Vector3.Zero, Eigen::Quaternion.Identity)
            end
        end

        # Enumerates the static transformations
        #
        # @yieldparam [StaticTransform] trsf
        def each_static_transform
            transforms.each_value do |val|
                yield(val) if val.kind_of?(StaticTransform)
            end
        end

        # Enumerates the dynamic transformations
        #
        # @yieldparam [DynamicTransform] trsf
        def each_dynamic_transform
            transforms.each_value do |val|
                yield(val) if val.kind_of?(DynamicTransform)
            end
        end

        # Enumerates the example transformations
        #
        # @yieldparam [StaticTransform] trsf
        def each_example_transform(&block)
            example_transforms.each_value(&block)
        end

        # Tests whether some joints are registered
        def has_joints?
            !joints.empty?
        end

        # Registers a joint object
        #
        # @param [String] from the transformation's source frame
        # @param [String] to the transformation's target frame
        # @param [[#name,#transform_for]] joint the joint object. It should
        #   allow to compute the joint transformation (with transform_for) and to
        #   return its name
        # @return [void]
        def register_joint(from, to, joint)
            joints << [from, to, joint]
        end

        # Enumerates the joints registered on this configuration
        #
        # @yieldparam [String] from the transformation's source frame
        # @yieldparam [String] to the transformation's target frame
        # @yieldparam [[#name,#transform_for]] joint the joint object. It should
        #   allow to compute the joint transformation (with transform_for) and to
        #   return its name
        # @return [void]
        def each_joint(&block)
            joints.each(&block)
        end

        def pretty_print(pp)
            pp.text "Transformer configuration"
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

                pp.breakable
                pp.text "Example Transforms:"
                pp.nest(2) do
                    each_example_transform do |i|
                        pp.breakable
                        i.pretty_print(pp)
                    end
                end
            end
        end

    end
end

