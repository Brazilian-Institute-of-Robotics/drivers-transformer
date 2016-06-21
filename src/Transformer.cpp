#include <transformer/Transformer.hpp>
#include <Eigen/LU>
#include <Eigen/SVD>
#include <assert.h>
#include <base-logging/Logging.hpp>

namespace transformer {
    
TransformationStatus Transformation::getStatus() const
{
    TransformationStatus status;
    updateStatus(status);
    return status;
}

void Transformation::updateStatus(TransformationStatus& status) const
{
    if (status.source_local != sourceFrame)
        status.source_local = sourceFrame;
    if (status.target_local != targetFrame)
        status.target_local = targetFrame;
    if (status.source_global != getSourceFrame())
        status.source_global = getSourceFrame();
    if (status.target_global != getTargetFrame())
        status.target_global = getTargetFrame();
    status.last_generated_value = lastGeneratedValue;
    status.chain_length = transformationChain.size();
    status.generated_transformations = generatedTransformations;
    status.failed_no_sample = failedNoSample;
    status.failed_no_chain = failedNoChain;
    status.failed_interpolation_impossible = failedInterpolationImpossible;
}

void Transformation::setTransformationChain(const std::vector< TransformationElement* >& chain)
{
    transformationChain = chain;
    valid = true;
    
    if(!transformationChangedCallback.empty())
    {
	for(std::vector< TransformationElement* >::iterator it = transformationChain.begin();
	it != transformationChain.end(); it++)
	{
            StaticTransformationElement *staticElem = dynamic_cast<StaticTransformationElement *>(*it);;
            InverseTransformationElement *invElem = dynamic_cast<InverseTransformationElement *>(*it);
            if(invElem)
            {
                staticElem = dynamic_cast<StaticTransformationElement *>(invElem->getElement());
            }
            if(staticElem)
            {
                //call the callback, as the transformation will never 'change' again 
                transformationChangedCallback(base::Time());
            }
            else
                (*it)->addTransformationChangedCallback(transformationChangedCallback);
	}
    }
}

void Transformation::registerUpdateCallback(boost::function<void (const base::Time &ts)> callback)
{
    transformationChangedCallback = callback;
    
    if(valid)
    {
	for(std::vector< TransformationElement* >::iterator it = transformationChain.begin();
	it != transformationChain.end(); it++)
	{
	    (*it)->addTransformationChangedCallback(transformationChangedCallback);
	}
    }
}

class TransformationNode {
    public:
	TransformationNode() : parent(NULL) {};
	TransformationNode(const std::string &frameName, TransformationNode *parent, TransformationElement *parentToCurNode) : frameName(frameName), parent(parent), parentToCurNode(parentToCurNode) {};
	
	std::string frameName;
	TransformationNode *parent;
	TransformationElement *parentToCurNode;
	std::vector<TransformationNode *> childs;
	~TransformationNode() {
	    //delete all known childs
	    for(std::vector<TransformationNode *>::iterator it = childs.begin(); it != childs.end(); it++) {
		delete *it;
	    }
	    childs.clear();
	}
};

TransformationTree::~TransformationTree()
{
    clear();
}

std::pair<int, int> TransformationTree::getElementsCount() const
{
    int static_count = 0, dynamic_count = 0;
    for(std::vector< TransformationElement* >::const_iterator it = availableElements.begin(); it != availableElements.end(); it++)
    {
        TransformationElement const* element = *it;
        InverseTransformationElement const* inv_element = dynamic_cast<InverseTransformationElement const*>(*it);
        if (inv_element)
            element = inv_element->getElement();

        if (dynamic_cast<DynamicTransformationElement const*>(element))
            dynamic_count++;
        else
            static_count++;
    }
    return std::make_pair(static_count, dynamic_count);
}

void TransformationTree::dumpTree() const
{
    for(std::vector< TransformationElement* >::const_iterator it = availableElements.begin(); it != availableElements.end(); it++)
    {
        TransformationElement const* element = *it;
        bool is_inv = false, is_dyn = false;

        InverseTransformationElement const* inv_element = dynamic_cast<InverseTransformationElement const*>(*it);
        if (inv_element)
        {
            is_inv = true;
            element = inv_element->getElement();
        }

        if (dynamic_cast<DynamicTransformationElement const*>(element))
            is_dyn = true;


        if (is_inv)
        {
            if (is_dyn)
                LOG_DEBUG_S << "(inv,dyn) " << inv_element->getSourceFrame() << " > " << inv_element->getTargetFrame() << std::endl;
            else
                LOG_DEBUG_S << "(inv,static) " << inv_element->getSourceFrame() << " > " << inv_element->getTargetFrame() << std::endl;
        }
        else
        {
            if (is_dyn)
                LOG_DEBUG_S << "(dyn) " << element->getSourceFrame() << " > " << element->getTargetFrame() << std::endl;
            else
                LOG_DEBUG_S << "(static) " << element->getSourceFrame() << " > " << element->getTargetFrame() << std::endl;
        }
    }
}

void TransformationTree::addTransformation(TransformationElement* element)
{
    //add transformation
    availableElements.push_back(element);
    
    //and it's inverse
    TransformationElement *inverse = new InverseTransformationElement(element);
    availableElements.push_back(inverse);
    
}

void TransformationTree::addMatchingTransforms(std::string from, TransformationNode *node)
{
    for(std::vector< TransformationElement* >::const_iterator it = availableElements.begin(); it != availableElements.end(); it++)
    {
	if((*it)->getSourceFrame() == from)
	{
	    //security check for not building A->B->A->B loops
	    if(node->parent && node->parent->frameName == (*it)->getTargetFrame())
		continue;
	    
	    node->childs.push_back(new TransformationNode((*it)->getTargetFrame(), node, *it));
	}	
    }
}

std::vector< TransformationNode* >::const_iterator TransformationTree::checkForMatchingChildFrame(const std::string& to, const transformer::TransformationNode& node)
{
    for(std::vector<TransformationNode *>::const_iterator it = node.childs.begin(); it != node.childs.end(); it++)
    {
	if((*it)->frameName == to)
	    return it;
    }
    
    return node.childs.end();
}


bool TransformationTree::getTransformationChain(std::string from, std::string to, std::vector< TransformationElement* >& result)
{
    if (from == to)
        return true;

    TransformationNode node(from, NULL, NULL);
    
    std::vector<TransformationNode *> curLevel;
    curLevel.push_back(&node);
    
    for(int i = 0; i < maxSeekDepth && curLevel.size(); i++) {
	std::vector<TransformationNode *> nextLevel;

	for(std::vector<TransformationNode *>::iterator it = curLevel.begin(); it != curLevel.end(); it++)
	{
	    //expand tree node
	    addMatchingTransforms((*it)->frameName, *it);
	    
	    //check if a child of the node matches the wanted frame
	    std::vector< TransformationNode* >::const_iterator candidate = checkForMatchingChildFrame(to, **it);
	    if(candidate != (*it)->childs.end())
	    {
		LOG_DEBUG_S << "Found Transformation chain from " << from << " to " << to;
                LOG_DEBUG_S << "Chain is (reverse) : ";
		
		TransformationNode *curNode = *candidate;
		result.reserve(i + 1);
		
		//found a valid transformation
		while(curNode->parent)
		{
		    result.push_back(curNode->parentToCurNode);
		    LOG_DEBUG_S << "   " << curNode->frameName << " " << curNode->parentToCurNode->getTargetFrame() << "<->" << curNode->parentToCurNode->getSourceFrame();
		    
		    curNode = curNode->parent;
		}
		LOG_DEBUG_S << "   " << curNode->frameName << std::endl;
		
		return true;
	    }
	    
	    //add childs of current level to search area for next level
	    nextLevel.insert(nextLevel.end(), (*it)->childs.begin(), (*it)->childs.end());
	}
	
	curLevel = nextLevel;
    }
    
    LOG_DEBUG_S << "could not find result for " << from << " " << to;

    return false;
}

TransformationElement const* InverseTransformationElement::getElement() const
{
    return nonInverseElement;
}

TransformationElement* InverseTransformationElement::getElement()
{
    return nonInverseElement;
}

bool InverseTransformationElement::getTransformation(const base::Time& atTime, bool doInterpolation, transformer::TransformationType& tr)
{
    if(nonInverseElement->getTransformation(atTime, doInterpolation, tr)){
	Eigen::Affine3d tr2(Eigen::Affine3d::Identity());
	tr2 = tr;
	tr2 = tr2.inverse();
	tr.setTransform(tr2);
        std::swap(tr.sourceFrame, tr.targetFrame);
	return true;
    }
    return false;
};

DynamicTransformationElement::DynamicTransformationElement(const std::string& sourceFrame, const std::string& targetFrame, aggregator::StreamAligner& aggregator, int priority )
    : TransformationElement(sourceFrame, targetFrame), aggregator(aggregator), gotTransform(false) 
{
    //giving a buffersize of zero means no buffer limitation at all
    //giving a period of zero means, block until next sample is available
    streamIdx = aggregator.registerStream<TransformationType>(
	    boost::bind( &transformer::DynamicTransformationElement::aggregatorCallback , this, _1, _2 ), 
	    0, base::Time(), priority, sourceFrame + std::string("2") + targetFrame);
}

DynamicTransformationElement::~DynamicTransformationElement()
{
    aggregator.unregisterStream(streamIdx);
}

void DynamicTransformationElement::aggregatorCallback(const base::Time& ts, const transformer::TransformationType& value)
{
    gotTransform = true;
    lastTransform = value;
    lastTransformTime = ts;
    for(std::vector<boost::function<void (const base::Time &ts)> >::const_iterator it = elementChangedCallbacks.begin();
    it != elementChangedCallbacks.end(); it++)
    {
	(*it)(ts);
    }
}

bool DynamicTransformationElement::getTransformation(const base::Time& atTime, bool doInterpolation, transformer::TransformationType& result)
{
    if(!gotTransform)
    {
	//no sample available, return
	return false;
    }
    
    if(doInterpolation)
    {
	double timeForward = (atTime - lastTransformTime).toSeconds();
	
	if(timeForward < 0) 
	{
	    throw std::runtime_error("Error, time of sample is lower than transformation time"); 
	}
	
	if(timeForward == 0) 
	{
	    //transform time is equal to sample time, no interpolation needed
	    result = lastTransform;

	    return true;
	}

	std::pair<base::Time, TransformationType> next_sample;
	
	if(!aggregator.getNextSample(streamIdx, next_sample))
	{
	    //std::cout << "Transformer: could not get next sample" << std::endl;
	    //not enought samples for itnerpolation available
	    return false;
	}
	
	TransformationType interpolated;
	interpolated.initSane();

	double timeBetweenTransforms = (next_sample.first - lastTransformTime).toSeconds();

	assert(timeBetweenTransforms > timeForward);
	
	double factor = timeForward / timeBetweenTransforms;
	
	Eigen::Quaterniond start_r(lastTransform.orientation);
	Eigen::Quaterniond end_r(next_sample.second.orientation);
	
	interpolated.orientation = (start_r.slerp(factor, end_r));
	
	Eigen::Vector3d start_t(lastTransform.position);
	Eigen::Vector3d end_t(next_sample.second.position);
	
	interpolated.position = factor * start_t + (1.0-factor) * end_t; 

	// perform linear interpolation of uncertainties
	interpolated.cov_position = 
	    factor * lastTransform.cov_position + 
	    (1.0-factor) * next_sample.second.cov_position;

	interpolated.cov_orientation = 
	    factor * lastTransform.cov_orientation + 
	    (1.0-factor) * next_sample.second.cov_orientation;

	result = interpolated;
    } else {
	result = lastTransform;
    }

    return true;
};

void TransformationTree::clear()
{
    for(std::vector<TransformationElement *>::iterator it = availableElements.begin(); it != availableElements.end(); it++)
    {
	delete *it;
    }
    
    availableElements.clear();
}


bool Transformation::get(const base::Time &time, TransformationType& tr, bool doInterpolation) const
{
    tr.initSane();
    tr.sourceFrame = sourceFrame;
    tr.targetFrame = targetFrame;
    tr.time = time;

    Eigen::Affine3d fullTransformation;
    bool ret = get(time, fullTransformation, doInterpolation);
    if(!ret)
	return false;
    
    tr.setTransform(fullTransformation);
    return true;
}

bool Transformation::getChain(const base::Time& time, std::vector< TransformationType >& tr, bool doInterpolation) const
{
    if(transformationChain.empty()) 
    {
	return false;
    }

    tr.resize(transformationChain.size());

    TransformationType transform;
    
    int i = 0;
    
    for(std::vector<TransformationElement *>::const_iterator it = transformationChain.begin(); it != transformationChain.end(); it++)
    {
	tr[i].sourceFrame = (*it)->getSourceFrame();
	tr[i].targetFrame = (*it)->getTargetFrame();
	tr[i].time = time;
	if(!(*it)->getTransformation(time, doInterpolation, tr[i]))
	{
	    //no sample available, return
	    return false;
	}
	
	i++;
    }

    return true;
}

bool Transformation::getChain(const base::Time& atTime, std::vector< Eigen::Affine3d >& result, bool interpolate) const
{
    std::vector< TransformationType > intern;
    bool ret = getChain(atTime, intern, interpolate);
    if(!ret)
	return false;
    
    result.resize(intern.size());
    std::vector<Eigen::Affine3d >::iterator it_out = result.begin();
    for(std::vector<TransformationType >::const_iterator it = intern.begin(); it != intern.end(); it++)
    {
	*it_out = *it;
	it_out++;
    }
    
    return true;
}

void Transformation::setFrameMapping(const std::string& frameName, const std::string& newName)
{
    if(sourceFrame == frameName)
	sourceFrameMapped = newName;
    
    if(targetFrame == frameName)
	targetFrameMapped = newName;
}


Transformation& Transformer::registerTransformation(std::string sourceFrame, std::string targetFrame)
{
    Transformation *ret = new Transformation(sourceFrame, targetFrame);
    transformations.push_back(ret);
    
    std::vector< TransformationElement* > trChain;
    
    //check if a transformation chain for this transformation exists
    if(transformationTree.getTransformationChain(ret->getSourceFrame(), ret->getTargetFrame(), trChain))
    {
	ret->setTransformationChain(trChain);
    }
    
    return *ret;
}

void Transformer::unregisterTransformation(Transformation* transformation)
{
    std::vector<Transformation *>::iterator it = std::find(transformations.begin(), transformations.end(), transformation);
    if(it == transformations.end())
        throw std::runtime_error("Tried to unregister non existing transformation");

    transformations.erase(it);
    delete transformation;
}

void Transformer::recomputeAvailableTransformations()
{
    std::vector<TransformationElement *> &elements(transformationTree.getAvailableElements());
    //clear all currenty registered callbacks
    for(std::vector<TransformationElement *>::iterator element = elements.begin(); element != elements.end(); element++)
    {
        (*element)->clearTransformationChangedCallbacks();
    }
    
    //seek through all available data streams and update transformation chains
    for(std::vector<Transformation *>::iterator transform = transformations.begin(); transform != transformations.end(); transform++)
    {
	std::vector< TransformationElement* > trChain;
	
	if(transformationTree.getTransformationChain((*transform)->getSourceFrame(), (*transform)->getTargetFrame(), trChain))
	{
	    (*transform)->setTransformationChain(trChain);
	}
    }
}

void Transformer::pushDynamicTransformation(const transformer::TransformationType& tr)
{
    if(tr.sourceFrame == "" || tr.targetFrame == "")
	throw std::runtime_error("Dynamic transformation with empty target or source frame given");

    if(tr.time.isNull())
	throw std::runtime_error("Dynamic transformation without time given (or it is 1970 ;-P)");

    std::map<std::pair<std::string, std::string>, int>::iterator it = transformToStreamIndex.find(std::make_pair(tr.sourceFrame, tr.targetFrame));
    
    //we got an unknown transformation
    if(it == transformToStreamIndex.end()) {

	//create a representation of the dynamic transformation
	DynamicTransformationElement *dynamicElement = new DynamicTransformationElement(tr.sourceFrame, tr.targetFrame, aggregator, priority);
	
	int streamIdx = dynamicElement->getStreamIdx();
	
	transformToStreamIndex[std::make_pair(tr.sourceFrame, tr.targetFrame)] = streamIdx;
	
	LOG_DEBUG_S << "Registering new stream for transformation from " << tr.sourceFrame << " to " << tr.targetFrame << " index is " << streamIdx;
	
	//add new dynamic element to transformation tree
	transformationTree.addTransformation(dynamicElement);

	recomputeAvailableTransformations();
	
	it = transformToStreamIndex.find(std::make_pair(tr.sourceFrame, tr.targetFrame));
	assert(it != transformToStreamIndex.end());
    }

    //push sample
    aggregator.push(it->second, tr.time, tr);
}

void Transformer::pushStaticTransformation(const transformer::TransformationType& tr)
{
    if(tr.sourceFrame == "" || tr.targetFrame == "")
	throw std::runtime_error("Static transformation with empty target or source frame given");
    
    transformationTree.addTransformation(new StaticTransformationElement(tr.sourceFrame, tr.targetFrame, tr));
    recomputeAvailableTransformations();
}

void Transformer::setFrameMapping(const std::string& frameName, const std::string& newName)
{
    for(std::vector<Transformation *>::iterator transform = transformations.begin(); transform != transformations.end(); transform++)
    {
	(*transform)->setFrameMapping(frameName, newName);
    }
    recomputeAvailableTransformations();
}
    
void Transformer::addTransformationChain(std::string from, std::string to, const std::vector< TransformationElement* >& chain)
{
    for(std::vector<Transformation *>::iterator it = transformations.begin();
	it != transformations.end(); it++) 
    {
	if((*it)->getSourceFrame() == from && (*it)->getTargetFrame() == to)
	{
	    (*it)->setTransformationChain(chain);
	}
    }
}

const TransformerStatus& Transformer::getTransformerStatus()
{
    transformerStatus.transformations.resize(transformations.size());
    size_t i = 0;
    for(std::vector<Transformation *>::const_iterator it = transformations.begin(); 
        it != transformations.end(); it++)
    {
        (*it)->updateStatus(transformerStatus.transformations[i]);
        i++;
    }
    transformerStatus.time = base::Time::now();
    return transformerStatus;
}


void Transformer::clear()
{
    //clear all known transformation chains
    for(std::vector<Transformation *>::iterator it = transformations.begin(); it != transformations.end(); it++)
    {
	(*it)->reset();
    }

    //clear index mapping
    transformToStreamIndex.clear();
    
    //clear transformation tree
    transformationTree.clear();
    
    transformerStatus.time = base::Time();
    transformerStatus.transformations.clear();
    
    //clear data samples in the aggregator
    aggregator.clear();
}
    
Transformer::~Transformer()
{
    for(std::vector<Transformation *>::iterator it = transformations.begin(); it != transformations.end(); it++)
    {
	delete *it;
    }
    transformations.clear();
}
    
}
