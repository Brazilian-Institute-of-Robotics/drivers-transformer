#include "NonAligningTransformer.hpp"
#include <base-logging/Logging.hpp>

transformer::NonAlignedDynamicTransformationElement::NonAlignedDynamicTransformationElement(const std::string& sourceFrame, const std::string& targetFrame): TransformationElement(sourceFrame, targetFrame)
{
    gotTransform = false;
}

bool transformer::NonAlignedDynamicTransformationElement::getTransformation(const base::Time& atTime, bool doInterpolation, transformer::TransformationType& result)
{
    if(doInterpolation)
        throw std::runtime_error("Interpolated Transformation on nonAlignedTransformer requested");
    
    if(!gotTransform)
        return false;
    
    result = lastTransform;
    
    return true;
}

void transformer::NonAlignedDynamicTransformationElement::setTransformation(const base::Time& atTime, const transformer::TransformationType& tr)
{
    gotTransform = true;
    lastTransform = tr;
    lastTransformTime = atTime;
    if(!elementChangedCallback.empty())
	elementChangedCallback(atTime);
}

transformer::NonAligningTransformer::NonAligningTransformer()
{

}

void transformer::NonAligningTransformer::clear()
{
    transformToElementMap.clear();
    transformer::Transformer::clear();
}


void transformer::NonAligningTransformer::pushDynamicTransformation(const transformer::TransformationType& tr)
{
    if(tr.sourceFrame == "" || tr.targetFrame == "")
        throw std::runtime_error("Dynamic transformation with empty target or source frame given");

    if(tr.time.isNull())
        throw std::runtime_error("Dynamic transformation without time given (or it is 1970 ;-P)");

    std::map<std::pair<std::string, std::string>, NonAlignedDynamicTransformationElement *>::iterator it = transformToElementMap.find(std::make_pair(tr.sourceFrame, tr.targetFrame));
    
    //we got an unknown transformation
    if(it == transformToElementMap.end()) {

        //create a representation of the dynamic transformation
        NonAlignedDynamicTransformationElement *dynamicElement = new NonAlignedDynamicTransformationElement(tr.sourceFrame, tr.targetFrame);
        
        transformToElementMap[std::make_pair(tr.sourceFrame, tr.targetFrame)] = dynamicElement;
        
        LOG_DEBUG_S << "Registering new stream for transformation from " << tr.sourceFrame << " to " << tr.targetFrame;
        
        //add new dynamic element to transformation tree
        transformationTree.addTransformation(dynamicElement);

        recomputeAvailableTransformations();
        
        it = transformToElementMap.find(std::make_pair(tr.sourceFrame, tr.targetFrame));
        assert(it != transformToElementMap.end());
    }

    //set new transformation
    it->second->setTransformation(tr.time, tr);
}
