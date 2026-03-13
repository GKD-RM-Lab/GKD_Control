#include <cstdio>
#include <ctime>

#include "Matrix.hpp"
#include "macro_helpers.hpp"

#pragma once

namespace Power
{
    namespace Math
    {

        template<uint32_t dim>
        class RLS
        {
           public:
            /**
             * @brief Delete the default constructor
             */
            RLS() = delete;

            /**
             * @brief The constructor
             * @param delta_ The intialized non-singular value of the transfer matrix
             * @param lambda_ The forgotten index
             */
            constexpr RLS(float delta_, float lambda_)
                : dimension(dim),
                  lambda(lambda_),
                  delta(delta_),
                  lastUpdate(0),
                  updateCnt(0),
                  defaultParamsVector(Matrixf<dim, 1>::zeros()) {
                this->reset();
                this->validate();
            }

            constexpr RLS(float delta_, float lambda_, Matrixf<dim, 1> initParam) : RLS(delta_, lambda_) {
                defaultParamsVector = initParam;
            }

            /**
             * @brief Reset the RLS module
             * @retval None
             */
            void reset() {
                transMatrix = Matrixf<dim, dim>::eye() * delta;
                gainVector = Matrixf<dim, 1>::zeros();
                paramsVector = Matrixf<dim, 1>::zeros();
            }

            /**
             * @brief Proccess a cycle of RLS update
             * @param sampleVector The new samples input expressed in n x 1 dimensionasl vector form
             * @param actualOutput The actual feedback real output
             * @retval paramsVector
             */
            const Matrixf<dim, 1> &update(Matrixf<dim, 1> &sampleVector, float actualOutput) {
                const float proj = (sampleVector.trans() * transMatrix * sampleVector)[0][0];
                const float gainDen = lambda + proj;
                if (!std::isfinite(gainDen) || fabsf(gainDen) < 1e-8f || !std::isfinite(actualOutput)) {
                    return paramsVector;
                }

                gainVector = (transMatrix * sampleVector) / gainDen;  // Get gain vector
                const float prediction = (sampleVector.trans() * paramsVector)[0][0];
                if (!std::isfinite(prediction)) {
                    return paramsVector;
                }
                paramsVector += gainVector * (actualOutput - prediction);  // Get params vector
                transMatrix =
                    (transMatrix - gainVector * sampleVector.trans() * transMatrix) / lambda;  // Get transferred matrix

                updateCnt++;
                lastUpdate = clock();
                return paramsVector;
            }

            /**
             * @brief Set the default regression parameters
             * @param updatedParams
             * @retval None
             */
            void setParamVector(const Matrixf<dim, 1> &updatedParams) {
                paramsVector = updatedParams;
                defaultParamsVector = updatedParams;
            }
            /**
             * @brief The getter function of the params vector
             * @param None
             * @retval paramsVector
             */
            constexpr Matrixf<dim, 1> &getParamsVector() const {
                return paramsVector;
            }

            /**
             * @brief The getter function of the output vector
             * @param None
             * @retval The estimated / filterd output of the RLS module
             */
            const float &getOutput() const {
                return output;
            }

           private:
            /**
             * @brief Lambda and delta validate check
             * @param None
             * @retval None
             */
            void validate() const {
                configASSERT(lambda > 0.0f && lambda <= 1.0f);
                configASSERT(delta > 0);
            }

            uint32_t dimension;  // Dimension of the RLS space
            float lambda;        // The forget index
            float delta;         // Intialized value of the transferred matrix

            size_t lastUpdate;   // Last update tick
            uint32_t updateCnt;  // Total update Count

            /*RLS relvant matrix*/
            Matrixf<dim, dim> transMatrix;  // Transfer matrix instance
            Matrixf<dim, 1> gainVector;     // Gain vector for params update
            Matrixf<dim, 1> paramsVector;   // Params vector
            Matrixf<dim, 1> defaultParamsVector;
            float output;  // Estimated / filtered output
        };
    }  // namespace Math
}  // namespace Power
