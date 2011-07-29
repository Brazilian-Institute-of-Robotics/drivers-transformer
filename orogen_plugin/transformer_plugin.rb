require 'aggregator_plugin'
module TransformerPlugin
    include AggregatorPlugin
    
    class TransformerGenerator
	attr_accessor :task
	attr_reader :transformer_name
	
	def initialize(task)
	    @task = task
	    @transformer_name = "transformer"
	end

	def generate_parse_time_code(config)
	    task.project.using_library('transformer', :typekit => false)

	    task.property("transformer_max_latency",   'double', config.max_latency).
		doc "Maximum time that should be waited for a delayed data sample to arrive"
	    Orocos::Generation.info("Adding property transformer_max_latency")

	    task.project.import_types_from('StreamAlignerStatus.hpp')

	    #add output port for status information
	    task.output_port("#{transformer_name}_status", '/aggregator/StreamAlignerStatus')
	    Orocos::Generation.info("Adding port #{transformer_name}_status")
	    
	    #add period property for every data stream
	    config.streams.each do |m|
		property_name = "#{m.name}_period"
		if(!(task.find_property(property_name)))
		    task.property(property_name,   'double', m.period).
			doc "Time in s between #{m.name} readings"
		    Orocos::Generation.info("Adding property #{property_name}")
		end
		
		#push data in update hook
		task.add_port_listener(m.name) do |sample_name|
		    "
	#{transformer_name}.pushData(#{m.idx_name}, #{sample_name}.time, #{sample_name});"
		end
	    end	    
	    
	    #create ports for transformations
	    task.input_port('static_transformations', '/base/samples/RigidBodyState')
	    task.input_port('dynamic_transformations', '/base/samples/RigidBodyState').
		needs_reliable_connection
		
	    task.hidden_operation("setFrameMapping", "    #{@transformer_name}.setFrameMapping(sourceFrame, mappedFrame);").
		argument('sourceFrame', '/std/string').
		argument('mappedFrame', '/std/string').
		doc("Mappes 'sourceFrame' to 'mappedFrame' for all defined Transformations. This method may be called multiple times.")

	end
	
	def generate_transformer_code(config)
	    task.add_base_header_code("#include<transformer/Transformer.h>", true)
	    #a_transformer to be shure that the transformer is declared BEFORE the Transformations
	    task.add_base_member("a_transformer", transformer_name, "transformer::Transformer")
	    task.add_base_member("lastStatusTime", "_lastStatusTime", "base::Time")
	    
	    task.in_base_hook("configure", "
    #{transformer_name}.clear();
    #{transformer_name}.setTimeout( base::Time::fromSeconds( _transformer_max_latency.value()) );
	    ")	    
	    
	    config.transformations.each do |t|
		task.add_base_member("transformer", t.name , "transformer::Transformation &").
		    initializer("#{t.name}(#{transformer_name}.registerTransformation(\"#{t.from}\", \"#{t.to}\"))")
	    end
	    
	    config.streams.each do |s|
		stream_data_type = s.get_data_type(task)
		
		#add variable for index
		task.add_base_member("transformer", s.idx_name, "int")
		
		#add callbacks
		task.add_user_method("void", s.callback_name, "const base::Time &ts, const #{stream_data_type} &#{s.name}_sample").
		body("    throw std::runtime_error(\"Transformer callback for #{s.name} not implemented\");")

		#register streams at transformer
		buffer_size_factor = 2.0
		task.in_base_hook("configure", "
    {
    const double #{s.name}Period = _#{s.name}_period.value();
    #{s.idx_name} = #{transformer_name}.registerDataStream< #{stream_data_type}>(
		    base::Time::fromSeconds(#{s.name}Period), boost::bind( &#{task.class_name()}Base::#{s.callback_name}, this, _1, _2));
    }
		")

		#unregister in cleanup
		task.in_base_hook("cleanup", "
    #{transformer_name}.unregisterDataStream(#{s.idx_name});")
	    end

	    task.in_base_hook("update", "
    base::samples::RigidBodyState staticTransforms;
    while(_static_transformations.read(staticTransforms, false) == RTT::NewData) {
	#{transformer_name}.pushStaticTransformation(staticTransforms);
    }
		                 
    base::samples::RigidBodyState dynamicTransform;
    while(_dynamic_transformations.read(dynamicTransform, false) == RTT::NewData) {
	#{transformer_name}.pushDynamicTransformation(dynamicTransform);
    }

    while(#{transformer_name}.step()) 
    {
	;
    }")

	    task.in_base_hook('update', "
    {
	const base::Time curTime(base::Time::now());
	if(curTime - _lastStatusTime > base::Time::fromSeconds(1))
	{
	    _lastStatusTime = curTime;
	    _#{transformer_name}_status.write(#{transformer_name}.getStatus());
	}
    }")

	    #unregister in cleanup
	    task.in_base_hook("stop", "
    #{transformer_name}.clear();")
	end
    end
    
    class TransformerConfiguration
	class StreamDescription
	    def initialize(name, period)
		@name = name
		@period = period
		@idx_name = name + "_idx_tr"
		@callback_name = name + "TransformerCallback"
	    end
	    attr_reader :name
	    attr_reader :period
	    attr_reader :idx_name
	    attr_reader :callback_name
	    
	    def get_data_type(task)
		port = task.find_port(name)
		if(!port)
		    raise "Error trying to register nonexisting port " + name + " to the transformer"
		end
	    
		port.type.cxx_name	    
	    end
	end
	
	class TransformationDescription
	    attr_reader :from
	    attr_reader :to
	    
	    attr_reader :name
	    
	    def initialize(from, to)
		@from = from
		@to = to
		@name = "_"+from+"2"+to 
	    end
	end
	
	dsl_attribute :max_latency
	attr_accessor :streams
	attr_accessor :transformations

	def initialize()
	    @streams = Array.new()
	    @transformations = Array.new()
	end
	
	def transformation(from, to)
	    transformations.push(TransformationDescription.new(from, to))
	end
	
	def align_port(name, period)
	    streams << StreamDescription.new(name, period)
	end

    end
    
    def transformer(&block)
	register_port_listener_generator()

	config = TransformerConfiguration.new()
	config.instance_eval(&block)

	if(!config.max_latency)
	   raise "not max_latency specified for transformer" 
	end
	
	gen = TransformerGenerator.new(self)
	gen.generate_parse_time_code(config)
	
	#register code generator to be called after parsing is done
	add_generation_handler do
	    gen.generate_transformer_code(config)
	end

    end
end


class Orocos::Generation::TaskContext
    include TransformerPlugin
end
