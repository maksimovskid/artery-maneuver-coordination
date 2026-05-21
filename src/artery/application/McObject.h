#ifndef ARTERY_MCOBJECT_H_
#define ARTERY_MCOBJECT_H_

#include <omnetpp/cobject.h>
#include <vanetza/asn1/mcm.hpp>

#include <memory>

namespace artery
{

using Mcm = vanetza::asn1::Mcm;

class McObject : public omnetpp::cObject
{
public:
    McObject(const McObject&) = default;
    McObject& operator=(const McObject&) = default;

    McObject(Mcm&&);
    McObject& operator=(Mcm&&);

    McObject(const Mcm&);
    McObject& operator=(const Mcm&);

    McObject(const std::shared_ptr<const Mcm>&);
    McObject& operator=(const std::shared_ptr<const Mcm>&);

    const Mcm& asn1() const;
    std::shared_ptr<const Mcm> shared_ptr() const;

    omnetpp::cObject* dup() const override;

private:
    std::shared_ptr<const Mcm> m_mcm_wrapper;
};

}  // namespace artery

#endif /* ARTERY_MCOBJECT_H_ */
