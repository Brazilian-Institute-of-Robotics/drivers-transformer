#ifndef TRANSFORMER_H
#define TRANSFORMER_H

#include <Eigen/Geometry>
#include <string>
#include <base/Time.hpp>
#include <aggregator/StreamAligner.hpp>
#include <map>
#include <boost/bind.hpp>
#include <base/samples/RigidBodyState.hpp>
#include "TransformationStatus.hpp"

namespace transformer {
 
typedef base::samples::RigidBodyState TransformationType;
class TransformationElement;

class Transformation
{
    friend class Transformer;
    private:
	Transformation(const std::string &sourceFrame, const std::string &targetFrame)
            : valid(false)
            , sourceFrame(sourceFrame)
            , targetFrame(targetFrame)
            , generatedTransformations(0)
            , failedNoChain(0)
            , failedNoSample(0)
            , failedInterpolationImpossible(0) {};
        bool valid;

	std::string sourceFrame;
	std::string targetFrame;
	std::string sourceFrameMapped;
	std::string targetFrameMapped;
	std::vector<TransformationElement *> transformationChain;

        mutable base::Time lastGeneratedValue;
        mutable uint64_t generatedTransformations;
        mutable uint64_t failedNoChain;
        mutable uint64_t failedNoSample;
        mutable uint64_t failedInterpolationImpossible;
	boost::function<void (const base::Time &ts)> transformationChangedCallback;
	
	void setFrameMapping(const std::string &frameName, const std::string &newName);
	
	/**
	 * Sets the transformation chain for this transformation
         *
         * The transformation chain is a list of links (represented by
         * TransformationElement objects) that should be composed to compute the
         * required transformation
         *
         * Calling this method sets the transformation as valid.
	 * */
	void setTransformationChain(const std::vector<TransformationElement *> &chain);
	
	Transformation(const Transformation &other)
	{
	}
	
    public:
        /** Updates the data contained in the provided status structure with the
         * transformation's internal information
         */
        TransformationStatus getStatus() const;

        /** Updates the data contained in the provided status structure with the
         * transformation's internal information
         */
        void updateStatus(TransformationStatus& status) const;

	/**
	 * returns the souce frame
	 * */
	const std::string &getSourceFrame() const
	{
	    if(sourceFrameMapped.empty())
		return sourceFrame;
	    
	    return sourceFrameMapped;
	}
	
	/**
	 * returns the target frame
	 * */
	const std::string &getTargetFrame() const
	{
	    if(targetFrameMapped.empty())
		return targetFrame;
	    
	    return targetFrameMapped;
	}	

        /** Clears all stored information and marks the transformation as
         * invalid
         */
        void reset()
        {
            valid = false;
            transformationChain.clear();
            lastGeneratedValue = base::Time();
            generatedTransformations = 0;
            failedNoChain = 0;
            failedNoSample = 0;
            failedInterpolationImpossible = 0;
        }

	/**
	 * Registeres a callback (only one) callback, that is called,
	 * whenever this transformation changes.
	 * */
	void registerUpdateCallback(boost::function<void (const base::Time &ts)> callback);
        
	/**
	 * This functions tries to return the transformation from sourceFrame to targetFrame at the given time.
	 * 
	 * If no chain, or no transformation samples are available the function will return false
	 * Else it will return true and store the requested transformation in @param result
	 * */
	bool get(const base::Time& atTime, transformer::TransformationType& result, bool interpolate = false) const;
	bool getChain(const base::Time& atTime, std::vector<TransformationType>& result, bool interpolate = false) const;

	template <class T>
	bool get(const base::Time& atTime, T& result, bool interpolate = false) const;
	bool getChain(const base::Time& atTime, std::vector<Eigen::Affine3d>& result, bool interpolate = false) const;
};

/**
 * This is a base class, that represens an abstract transformation from sourceFrame to targetFrame. 
 * */
class TransformationElement {
    public:
	TransformationElement(const std::string &sourceFrame, const std::string &targetFrame): sourceFrame(sourceFrame), targetFrame(targetFrame) {};
	virtual ~TransformationElement() {};
	
	/**
	 * This method my be asked for a concrete transformation at a time X.
	 * On request this should should interpolate the transformation to
	 * match the given time better than the available data. 
	 * 
	 * Note if no transformation is available getTransformation will return false
	 * */
	virtual bool getTransformation(const base::Time &atTime, bool doInterpolation, TransformationType &tr) = 0;

	/**
	 * This function registers a callback, that should be called every
	 * time the TransformationElement changes its value. 
	 * */
        virtual void addTransformationChangedCallback(boost::function<void (const base::Time &ts)> callback)
        {
            elementChangedCallbacks.push_back(callback);
        };
        
        /**
         * Removes all registered callbacks
         * */
        virtual void clearTransformationChangedCallbacks()
        {
            elementChangedCallbacks.clear();
        }
        
	/**
	 * returns the name of the source frame
	 * */
	const std::string &getSourceFrame() const
	{
	    return sourceFrame;
	}
	
	/**
	 * returns the name of the target frame
	 * */
	const std::string &getTargetFrame() const
	{
	    return targetFrame;
	}

    protected:
        std::vector<boost::function<void (const base::Time &ts)> > elementChangedCallbacks;
    private:
	std::string sourceFrame;
	std::string targetFrame;
};

/**
 * This class represents a static transformation
 * */
class StaticTransformationElement : public TransformationElement {
    public:
	StaticTransformationElement(const std::string &sourceFrame, const std::string &targetFrame, const TransformationType &transform) : TransformationElement(sourceFrame, targetFrame), staticTransform(transform) {};
	
	virtual bool getTransformation(const base::Time& atTime, bool doInterpolation, TransformationType& tr)
	{
	    tr = staticTransform;
            tr.time = atTime;
	    return true;
	};
    private:
	TransformationType staticTransform;
};

/**
 * This class represents a dynamic transformation
 * 
 * 
 * */
class DynamicTransformationElement : public TransformationElement {
    public:
	DynamicTransformationElement(const std::string& sourceFrame, const std::string& targetFrame, aggregator::StreamAligner& aggregator, int priority = -10);
	virtual ~DynamicTransformationElement();
	
	virtual bool getTransformation(const base::Time& atTime, bool doInterpolation, TransformationType& result);
        
	int getStreamIdx() const
	{
	    return streamIdx;
	}
	
    private:
	
	void aggregatorCallback(const base::Time &ts, const TransformationType &value); 

	aggregator::StreamAligner &aggregator;
	base::Time lastTransformTime;
	TransformationType lastTransform;
	bool gotTransform;
	int streamIdx;
};

/**
 * This class represents an inverse transformation.
 * */
class InverseTransformationElement : public TransformationElement {
    public:
	InverseTransformationElement(TransformationElement *source): TransformationElement(source->getTargetFrame(), source->getSourceFrame()), nonInverseElement(source) {};
	virtual bool getTransformation(const base::Time& atTime, bool doInterpolation, TransformationType& tr);

	virtual void addTransformationChangedCallback(boost::function<void (const base::Time &ts)> callback) 
	{
	    nonInverseElement->addTransformationChangedCallback(callback);
	};
	
        TransformationElement* getElement();
        TransformationElement const* getElement() const;
    private:
	TransformationElement *nonInverseElement;
};


class  TransformationNode;

/**
 * A class that can be used to get a transformation chain from a set of TransformationElements
 * */
class TransformationTree
{
    public:
	///default constructor
	TransformationTree() : maxSeekDepth(20) {};

        /** Returns the number of registered elements in the tree, as a (static
         * elements, dynamic elements) pair
         */
        std::pair<int, int> getElementsCount() const;
	
	/**
	 * Adds a TransformationElement to the set of available elements.
	 * 
	 * Note, internal the TransformationTree will also add an inverse
	 * transformation.
	 * */
	void addTransformation(TransformationElement *element);
	
	/**
	 * This function tries to generate a transformationChain from 'from' to 'to'.
	 * 
	 * To generate the transformationChain this function will spann a tree of
	 * transformations originating from 'from'. The function will then perform a
	 * breadth-first search until it either finds a chain, the tree can't be expanded
	 * further, or the search depth is deeper than maxSeekDepth.
	 * 
	 * In case a chain was found the function returns true and the chain is stored in result.
	 * */
	bool getTransformationChain(std::string from, std::string to, std::vector<TransformationElement *> &result);
	
        /**
         * Returns a vector of all currently registered transformation elements.
         * */
        std::vector<TransformationElement *> &getAvailableElements()
        {
            return availableElements;
        }
        
	/**
	 * Destructor, deletes all TransformationElements saved in availableElements
	 * */
	~TransformationTree();	
	
	/**
	 * Deletes all available TransformationElements
	 * */
	void clear();

        /** Dumps information about this tree on LOG_DEBUG_S
         */
        void dumpTree() const;
	
    private:
	///maximum seek depth while trying to find a transformationChain
	const int maxSeekDepth;

	/**
	 * This function will expand the given node.
	 * 
	 * To expand the node, the function will look up all available TransformationElements
	 * add new node for those, where the sourceFrame matches from
	 * */
	void addMatchingTransforms(std::string from, transformer::TransformationNode* node);
	
	/**
	 * Seeks through childs of the node and looks for a node that has frame 'to'
	 * 
	 * If found the function returns an iterator the the child.
	 * If not returns an iterator pointing to node.childs.end()
	 * */
	std::vector< TransformationNode* >::const_iterator checkForMatchingChildFrame(const std::string &to, const TransformationNode &node);
	
	/// List of available transformation elements
	std::vector<TransformationElement *> availableElements;
};

/**
 * A class that provides transformations to given samples, ordered in time.
 * */
class Transformer
{
    protected:
	aggregator::StreamAligner aggregator;
	std::map<std::pair<std::string, std::string>, int> transformToStreamIndex;
	std::vector<Transformation *> transformations;
	TransformationTree transformationTree;
	int priority;
        TransformerStatus transformerStatus;

	void recomputeAvailableTransformations();
	
    public:
	
	/** 
	 * Default constructor for a transformer
	 *
	 * @param priority - stream priority which is given to dynamic transform streams.
	 */
	Transformer( int priority = -10 ) 
	    : priority( priority ) {};
	
	/**
	 * Deletes all dynamic and static transformations
	 * that are known to the transformer.
	 * 
	 * Also deletes all samples in the data streams
	 * 
	 * Callbacks and setups for the data streams are NOT deleted by this method.
	 * */
	virtual void clear();
	
	/**
	 * This function may be used to manually set a transformation chain.
	 * 
	 * The function seeks through all registered sample streams, and sets the given 
	 * chain to those streams, where the source and target frame matches.
	 * */
	void addTransformationChain(std::string from, std::string to, const std::vector<TransformationElement *> &chain);
	
	/**
	 * This function registers a wanted transformation at the transformation stack.
	 * 
	 * It returns a reference to an object that represents the wanted transformation.
	 * 
	 * Note, the time of the transformation advances if one calls step() on the
	 * transformer.
	 * */
	Transformation &registerTransformation(std::string sourceFrame, std::string targetFrame);

        /**
         * Returns a vector of the registered transformations.
         * */
        const std::vector<Transformation *> &getRegisteredTransformations()
        {
            return transformations;
        }

        /**
         * Unregisteres a transformation from the transfromation stack.
         * 
         * This removes and deletes the given transformation
         * */
        void unregisterTransformation(Transformation *transformation);
        
	/**
	 * Registers a callback that will be called every time a new transformation is available 
	 * for the given Transformation handle.
	 * 
	 * */
	void registerTransformCallback(Transformation &transform , boost::function<void (const base::Time &ts, const Transformation &t)> callback) 
	{
	    transform.registerUpdateCallback(boost::bind( callback, _1, boost::ref(transform) ));
	}

	/**
	 * This function registes a new data stream together with an callback. 
	 * 
	 * The callback will be called every time a new data sample is available.
	 * */
	template <class T> int registerDataStream(base::Time dataPeriod, boost::function<void (const base::Time &ts, const T &value)> callback, int priority = -1, const std::string &name = std::string())
	{
	    return aggregator.registerStream<T>(boost::bind( callback, _1, _2), 0, dataPeriod, priority, name);
	};

	/**
	 * This function registes a new data stream together with an callback. 
	 * 
	 * The callback will be called every time a new data sample is available.
	 * */
	template <class T> int registerDataStreamWithTransform(base::Time dataPeriod, Transformation &transformation, boost::function<void (const base::Time &ts, const T &value, const Transformation &t)> callback, int priority = - 1, const std::string &name = std::string())
	{
	    return aggregator.registerStream<T>(boost::bind( callback, _1, _2, boost::ref(transformation) ), 0, dataPeriod, priority, name);
	};

	/**
	 * This function unregistes a data stream. 
	 * */
	void unregisterDataStream(int idx)
	{
	    return aggregator.unregisterStream(idx);
	};

	/** 
	 * @copydoc aggregator::StreamAligner::disableStream
	 */
	void disableStream( int idx )
	{
	    return aggregator.disableStream( idx );
	}

	/** 
	 * @copydoc aggregator::StreamAligner::enableStream
	 */
	void enableStream( int idx )
	{
	    return aggregator.enableStream( idx );
	}

	/** 
	 * @copydoc aggregator::StreamAligner::isStreamActive
	 */
	bool isStreamActive( int idx ) const 
	{
	    return aggregator.isStreamActive( idx );
	}
	
	void requestTransformationAtTime(int idx, base::Time ts)
	{
	    aggregator.push(idx, ts, false);
	};
	
	/** Push new data into the stream
	 * @param ts - the timestamp of the data item
	 * @param data - the data added to the stream
	 */
	template <class T> void pushData( int idx,const base::Time &ts, const T& data )
	{
	    aggregator.push(idx, ts, data);
	};
	
	/**
	 * Process data streams, this basically calls StreamAligner::step().
	 * */
	int step() {
	    return aggregator.step();
	}
	
	/**
	 * Get debug output of underlying stream aligner
	 * */
	const aggregator::StreamAlignerStatus &getStreamAlignerStatus()
	{
	    return aggregator.getStatus();
	}
	
	void setTimeout(const base::Time &t )
	{
	    aggregator.setTimeout(t);
	}
	
	/**
	 * Function for adding new Transformation samples.
	 *
	 * This function will interally keep track of available 
	 * transformations and register streams for 'new'
	 * transformations.  
	 * */
	virtual void pushDynamicTransformation(const TransformationType &tr);
	
	/**
	 * Function for adding static Transformations.
	 * */
	void pushStaticTransformation(const TransformationType &tr);

	
	void setFrameMapping(const std::string &frameName, const std::string &newName);

	/** 
	 * @return the status of the StreamAligner, which contains current latency
	 * and buffer fill sizes of the individual streams.
	 */
	const aggregator::StreamAlignerStatus& getStatus()
	{
	    return aggregator.getStatus();
	}
	
        /** 
         * @return the status of the transformer
         */
        const TransformerStatus& getTransformerStatus();
	
	/**
	 * Destructor, deletes the TransformationMakers
	 * */
	virtual ~Transformer();
};

template<class T>
bool Transformation::get(const base::Time& atTime, T& result, bool interpolate) const
{
    result = T::Identity();
    if (!valid)
    {
        failedNoChain++;
        return false;
    }

    for(std::vector<TransformationElement *>::const_iterator it = transformationChain.begin(); it != transformationChain.end(); it++)
    {
	TransformationType tr;
	if(!(*it)->getTransformation(atTime, interpolate, tr))
	{
            if (interpolate)
                failedInterpolationImpossible++;
            else
                failedNoSample++;

	    //no sample available, return
	    return false;
	}
	
	//TODO, this might be a costly operation
	T trans( tr );
	
	//apply transformation
	result = result * trans;
    }
    lastGeneratedValue = atTime;
    generatedTransformations++;
    return true;
}

}
#endif // TRANSFORMER_H
