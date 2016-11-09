#include "../include/BeliefMap.h"
#include "../include/Sensor.h"
#include "../include/PixelSensor.h"
#include "../include/StereoCameraSensor.h"

#include <cassert>

class registerTreeType;

BeliefMap::BeliefMap() : octomap::OcTreeBaseImpl<BeliefVoxel, octomap::AbstractOcTree>(Parameters::voxelSize),
                         QVoxelMap(this)
{
    octomap::AbstractOcTree::registerTreeType(this);

    init();

    for (unsigned int x = 0; x < Parameters::voxelsPerDimensionX; ++x)
    {
        for (unsigned int y = 0; y < Parameters::voxelsPerDimensionY; ++y)
        {
            for (unsigned int z = 0; z < Parameters::voxelsPerDimensionZ; ++z)
            {
                octomap::point3d point(Parameters::xMin + x * Parameters::voxelSize,
                                       Parameters::yMin + y * Parameters::voxelSize,
                                       Parameters::zMin + z * Parameters::voxelSize);
                Belief belief;
                this->updateNode(point, belief);
            }
        }
    }

    ROS_INFO("Belief map has %d nodes in total.", (int) calcNumNodes());
    calcMinMax();

    ROS_INFO("Belief map range: (%.2f %.2f %.2f) to (%.2f %.2f %.2f)", min_value[0], min_value[1], min_value[2],
             max_value[0], max_value[1], max_value[2]);
}

std::string BeliefMap::getTreeType() const
{
    return "BeliefMap";
}

BeliefMap *BeliefMap::create() const
{
    return new BeliefMap();
}

Belief *BeliefMap::belief(const octomap::OcTreeKey &key) const
{
    BeliefVoxel *voxel = search(key);
    if (voxel != NULL)
        return voxel->getValue().get();

#ifdef LOG_DETAILS
    ROS_WARN_STREAM("Belief voxel at " << keyToCoord(key) << " could not be found.");
#endif
    return NULL;
}

std::valarray<Parameters::NumType> BeliefMap::bouncingProbabilitiesOnRay(const octomap::KeyRay &ray) const
{
    std::valarray<Parameters::NumType> bouncingProbabilities(ray.size());
    unsigned int i = 0;
    for (auto &key : ray)
    {
        auto *b = belief(key);
        if (b == NULL)
            bouncingProbabilities[i] = 0.5; // TODO correct error handling?
        else
            bouncingProbabilities[i] = b->mean();
        ++i;
    }
    return bouncingProbabilities;
}

std::valarray<Parameters::NumType> BeliefMap::reachingProbabilitiesOnRay(
        const octomap::KeyRay &ray,
        const std::valarray<Parameters::NumType> &bouncingProbabilities) const
{
    assert(ray.size() == bouncingProbabilities.size());
    std::valarray<Parameters::NumType> reachingProbabilities(ray.size());
    reachingProbabilities[0] = (Parameters::NumType) (1. - Parameters::spuriousMeasurementProbability);
    for (unsigned int i = 1; i < ray.size(); ++i)
    {
        reachingProbabilities[i] = (Parameters::NumType) (reachingProbabilities[i - 1] *
                                                          (1. - bouncingProbabilities[i - 1]));
    }
    return reachingProbabilities;
}

bool BeliefMap::update(const Observation &observation)
{
    lastUpdatedVoxels.clear();
    int fails = 0;
    for (auto &measurement : observation.measurements())
    {
#ifdef FAKE_2D
        StereoCameraSensor cameraSensor(measurement.sensor->position(), measurement.sensor->orientation());
        icm = cameraSensor.computeInverseCauseModel(measurement, *this);
#else
        icm = measurement.sensor->computeInverseCauseModel(measurement, *this);
#endif
        if (!icm)
        {
            ++fails;
            continue;
        }
        unsigned int i = 0;
        Parameters::NumType prBeforeVoxel = 0;
        Parameters::NumType prOnVoxel;
        Parameters::NumType prAfterVoxel = icm->posteriorOnRay.sum() + icm->posteriorInfinity;
        for (auto &key : icm->ray)
        {
            prOnVoxel = icm->posteriorOnRay[i];
            prAfterVoxel -= prOnVoxel;

            const QBeliefVoxel qBeliefVoxel = query(key);
            const BeliefVoxel *beliefVoxel = qBeliefVoxel.node();
            if (beliefVoxel == NULL)
            {
#ifdef LOG_DETAILS
                ROS_WARN("Belief voxel for given voxel key on ray could not be found.");
#endif
                break;
            }

            if (!beliefVoxel->getValue().get()->isBeliefValid())
            {
                ROS_ERROR("Belief voxel has invalid PDF before updating with measurement.");
                return false;
            }

            Parameters::NumType mean = beliefVoxel->getValue().get()->mean();
            Parameters::NumType a = (Parameters::NumType) ((1. - mean) * prOnVoxel - mean * prAfterVoxel);
            Parameters::NumType b = (Parameters::NumType) (
                    mean * (1. - mean) * (Parameters::spuriousMeasurementProbability + prBeforeVoxel) +
                    mean * prAfterVoxel);
            beliefVoxel->getValue().get()->updateBelief(a, b);

            if (!beliefVoxel->getValue().get()->isBeliefValid())
            {
                // already asserted in Belief::updateBelief
                ROS_ERROR("Belief voxel has invalid PDF after updating with measurement.");
                return false;
            }

#ifdef LOG_DETAILS
            auto meanAfter = beliefVoxel->getValue().get()->mean();
            ROS_INFO("Voxel %d/%d updated. Mean before: %f    Mean after: %f", (int) i + 1, (int) icm->rayLength, mean,
                     meanAfter);
#endif

            lastUpdatedVoxels.push_back(qBeliefVoxel);

            prBeforeVoxel += prOnVoxel;
            ++i;
            if (i >= icm->rayLength)
                break;
        }
        delete icm;
        icm = NULL;
    }
    if (fails > 0)
    {
        ROS_WARN("ICM Computation failed for %i of %i measurements.", fails, (int)observation.measurements().size());
    }
    updateVisualization();

    return true;
}

BeliefVoxel *BeliefMap::updateNode(const octomap::point3d &position, const Belief &belief)
{
    octomap::OcTreeKey key;
    if (!coordToKeyChecked(position, key))
        return NULL;
    return updateNode(key, belief);
}

BeliefVoxel *BeliefMap::updateNode(const octomap::OcTreeKey &key, const Belief &belief)
{
    BeliefVoxel *leaf = this->search(key);
    if (leaf != NULL)
    {
        leaf->setValue(std::make_shared<Belief>(belief));
        return leaf;
    }

    bool createdRoot = false;
    if (this->root == NULL)
    {
        this->root = new BeliefVoxel();
        this->tree_size++;
        createdRoot = true;
    }

    return _updateNodeRecurs(this->root, createdRoot, key, 0, belief);
}

BeliefVoxel *BeliefMap::_updateNodeRecurs(BeliefVoxel *node, bool node_just_created,
                                          const octomap::OcTreeKey &key,
                                          unsigned int depth, const Belief &belief)
{
    bool created_node = false;

    assert(node);

    // follow down to last level
    if (depth < this->tree_depth)
    {
        unsigned int pos = computeChildIdx(key, this->tree_depth - 1 - depth);
        if (!node->childExists(pos))
        {
            // child does not exist, but maybe it's a pruned node?
            if (!node->hasChildren() && !node_just_created)
            {
                // current node does not have children AND it is not a new node
                // -> expand pruned node
                _expandNode(node);
            }
            else
            {
                // not a pruned node, create requested child
                _createNodeChild(node, pos);
                created_node = true;
            }
        }

        BeliefVoxel *result = _updateNodeRecurs(node->getChild(pos), created_node, key, depth + 1, belief);

        return result;
    }
    else
    {
        // at last level, updateVisualization node, end of recursion
        node->setValue(std::make_shared<Belief>(belief));
        return node;
    }
}

void BeliefMap::_expandNode(BeliefVoxel *node)
{
    assert(!node->hasChildren());

    for (unsigned int k = 0; k < 8; k++)
    {
        BeliefVoxel *newNode = _createNodeChild(node, k);
        newNode->setValue(std::make_shared<Belief>(*(node->getValue().get())));
    }
}

BeliefVoxel *BeliefMap::_createNodeChild(BeliefVoxel *node, unsigned int childIdx)
{
    assert(childIdx < 8);

    if (node->childExists(childIdx))
        return node->getChild(childIdx);

    tree_size++;
    size_changed = true;

    node->createChild(childIdx);

    return node->getChild(childIdx);
}