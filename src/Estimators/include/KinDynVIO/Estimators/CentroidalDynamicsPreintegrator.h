/**
 * @file CentroidalDynamicsPreintegrator.h
 * @authors Prashanth Ramadoss
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef KINDYNVIO_ESTIMATORS_CENTROIDAL_DYNAMICS_PREINTEGRATOR_H
#define KINDYNVIO_ESTIMATORS_CENTROIDAL_DYNAMICS_PREINTEGRATOR_H

#include <KinDynVIO/Factors/CentroidalDynamicsFactor.h>
#include <KinDynVIO/Estimators/IO.h>

#include <BipedalLocomotion/System/Advanceable.h>
#include <BipedalLocomotion/TextLogging/Logger.h>
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/Model/Model.h>

#include <Eigen/Dense>
#include <memory>

namespace KinDynVIO
{
namespace Estimators
{

template <typename PreintegratedMeasurements>
class CentroidalDynamicsPreintegrator : public BipedalLocomotion::System::Advanceable<ProprioceptiveInput,
                                                        CentroidalDynamicsPreintegratorOutput<PreintegratedMeasurements> >
{
protected:
    PreintegratorStatus m_status{PreintegratorStatus::IDLE};
    std::shared_ptr<iDynTree::KinDynComputations> m_kinDyn;
public:
    using Output = CentroidalDynamicsPreintegratorOutput<PreintegratedMeasurements>;
    virtual ~CentroidalDynamicsPreintegrator() = default;
    virtual bool initialize(std::weak_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler> handler) final
    {
        const std::string printPrefix{"[CentroidalDynamicsPreintegrator::initialize]"};
        BipedalLocomotion::log()->error("{} Use initialize(handler, model) method instead.", printPrefix);
        return false;
    }

    virtual bool initialize(std::weak_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler> handler,
                            const iDynTree::Model& robotModel) = 0;
    virtual bool setInput(const ProprioceptiveInput& input) = 0;

    virtual bool advance() = 0;
    virtual const Output& getOutput() const = 0;
    virtual bool isOutputValid() const = 0;

    virtual void startPreintegration(const double& prevTimeStamp = 0.0) = 0;
    virtual void stopPreintegration() = 0;

    virtual void resetIntegration() = 0;
};


class CentroidalDynamicsCumulativeBiasPreintegrator :
    public CentroidalDynamicsPreintegrator<KinDynVIO::Factors::PreintegratedCDMCumulativeBias>
{
public:
    CentroidalDynamicsCumulativeBiasPreintegrator();
    virtual ~CentroidalDynamicsCumulativeBiasPreintegrator();

    using Output = CentroidalDynamicsPreintegratorOutput<KinDynVIO::Factors::PreintegratedCDMCumulativeBias>;

    bool initialize(std::weak_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler> handler,
                    const iDynTree::Model& robotModel) final;

    bool setInput(const ProprioceptiveInput& input) final;
    virtual bool advance() final;

    // the following functions are all intended to be called
    // only after multiple calls to advance(),
    // relevant to the intermediate integration steps
    // between states at i-th and j-th timestamp, has been called
    // currently no internal check is available for this
    // and the user needs to be careful about these function calls
    // depending on the satisfaction of some conditional statements

    const Output& getOutput() const final;
    bool isOutputValid() const final;


    virtual void resetIntegration() final;
    virtual void resetIntegration(const gtsam::CDMBiasCumulative& bias);

    void startPreintegration(const double& prevTimeStamp  = 0.0) override;
    void stopPreintegration() override;

private:
    class Impl;
    std::unique_ptr<Impl> m_pimpl;
};

} // namespace Estimators
} // namespace KinDynVIO
#endif // KINDYNVIO_ESTIMATORS_CENTROIDAL_DYNAMICS_PREINTEGRATOR_H
