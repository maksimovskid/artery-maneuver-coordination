#include "artery/application/McObject.h"

#include <omnetpp.h>

#include <cassert>

namespace artery
{

Register_Abstract_Class(McObject)

McObject::McObject(Mcm&& mcm) : m_mcm_wrapper(std::make_shared<Mcm>(std::move(mcm)))
{
}

McObject& McObject::operator=(Mcm&& mcm)
{
    m_mcm_wrapper = std::make_shared<Mcm>(std::move(mcm));
    return *this;
}

McObject::McObject(const Mcm& mcm) : m_mcm_wrapper(std::make_shared<Mcm>(mcm))
{
}

McObject& McObject::operator=(const Mcm& mcm)
{
    m_mcm_wrapper = std::make_shared<Mcm>(mcm);
    return *this;
}

McObject::McObject(const std::shared_ptr<const Mcm>& ptr) : m_mcm_wrapper(ptr)
{
    assert(m_mcm_wrapper);
}

McObject& McObject::operator=(const std::shared_ptr<const Mcm>& ptr)
{
    m_mcm_wrapper = ptr;
    assert(m_mcm_wrapper);
    return *this;
}

std::shared_ptr<const Mcm> McObject::shared_ptr() const
{
    assert(m_mcm_wrapper);
    return m_mcm_wrapper;
}

const Mcm& McObject::asn1() const
{
    return *m_mcm_wrapper;
}

omnetpp::cObject* McObject::dup() const
{
    return new McObject { *this };
}

using namespace omnetpp;

class McmStationIdResultFilter : public cObjectResultFilter
{
protected:
    void receiveSignal(cResultFilter* prev, simtime_t_cref t, cObject* object, cObject* details) override
    {
        if (auto mcm = dynamic_cast<McObject*>(object)) {
            const auto id = mcm->asn1()->header.stationID;
            fire(this, t, id, details);
        }
    }
};

Register_ResultFilter("mcmStationId", McmStationIdResultFilter)

class McmGenerationDeltaTimeResultFilter : public cObjectResultFilter
{
protected:
    void receiveSignal(cResultFilter* prev, simtime_t_cref t, cObject* object, cObject* details) override
    {
        if (auto mcm = dynamic_cast<McObject*>(object)) {
            const auto genDeltaTime = mcm->asn1()->mcm.generationDeltaTime;
            fire(this, t, genDeltaTime, details);
        }
    }
};

Register_ResultFilter("mcmGenerationDeltaTime", McmGenerationDeltaTimeResultFilter)

}  // namespace artery
