/***********************************************************************

Copyright (c) 2018, KTH Royal Institute of Technology
All rights reserved.

Authors: Joshua Haustein <haustein@kth.se>

Based on OMPLPlanner.h by authors:
         Michael Koval <mkoval@cs.cmu.edu>
         Matthew Klingensmith <mklingen@cs.cmu.edu>
         Christopher Dellin <cdellin@cs.cmu.edu>

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met:

  Redistributions of source code must retain the above copyright
  notice, this list of conditions and the following disclaimer.

  Redistributions in binary form must reproduce the above copyright
  notice, this list of conditions and the following disclaimer in the
  documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*************************************************************************/

#ifndef OR_REDIRECTABLEOMPLPLANNER_H_
#define OR_REDIRECTABLEOMPLPLANNER_H_

#include <ompl/datastructures/NearestNeighborsGNATNoThreadSafety.h>
#include <ompl/geometric/SimpleSetup.h>
#include <openrave-core.h>
#include <openrave/planner.h>
#include <openrave/planningutils.h>

#include <or_ompl/OMPLPlannerParameters.h>
#include <or_ompl/StateSpaces.h>

namespace or_ompl {

typedef boost::function<ompl::base::Planner*(ompl::base::SpaceInformationPtr)> PlannerFactory;

class RedirectableOMPLPlanner : public OpenRAVE::PlannerBase {
public:
    RedirectableOMPLPlanner(OpenRAVE::EnvironmentBasePtr penv,
        PlannerFactory const& planner_factory);
    virtual ~RedirectableOMPLPlanner();

    virtual bool InitPlan(OpenRAVE::RobotBasePtr robot,
        PlannerParametersConstPtr params);
    virtual bool InitPlan(OpenRAVE::RobotBasePtr robot, std::istream& input);

    virtual OpenRAVE::PlannerStatus PlanPath(OpenRAVE::TrajectoryBasePtr ptraj);

    virtual PlannerParametersConstPtr GetParameters() const
    {
        return m_parameters;
    }

    bool GetTimes(std::ostream& sout, std::istream& sin) const;
    bool GetParameterValCommand(std::ostream& sout, std::istream& sin) const;
    // Define a couple of additional functions that allow placement planner to interact with motion planner
    // Get the goals that the planner has reached
    bool GetReachedGoals(std::ostream& sout, std::istream& sin) const;
    // Reset goals to new goals and stop planning for old ones
    bool ResetGoals(std::ostream& sout, std::istream& sin);
    // Add waypoints collision-free configurations that the planner might wanna use
    bool AddWaypoints(std::ostream& sout, std::istream& sin);
    // get all reached configurations
    bool GetReachedPoints(std::ostream& sout, std::istream& sin) const;

protected:
    const ompl::base::PlannerPtr& get_planner()
    {
        return m_planner;
    }

private:
    bool m_initialized;
    PlannerFactory m_planner_factory;
    OMPLPlannerParametersPtr m_parameters;
    ompl::geometric::SimpleSetupPtr m_simple_setup;
    ompl::base::StateSpacePtr m_state_space;
    OrStateValidityCheckerPtr m_or_validity_checker;
    ompl::base::PlannerPtr m_planner;
    OpenRAVE::RobotBasePtr m_robot;
    OpenRAVE::CollisionReportPtr m_collisionReport;
    double m_totalPlanningTime;
    // store current goals here to easily identify them when we have a solution
    struct StateWithId {
        ompl::base::ScopedState<ompl::base::StateSpace> state;
        unsigned int id;
        StateWithId(const ompl::base::ScopedState<ompl::base::StateSpace>& istate, unsigned int iid)
            : state(istate)
            , id(iid)
        {
        }
        static double distance(const boost::shared_ptr<StateWithId>& a, const boost::shared_ptr<StateWithId>& b)
        {
            assert(a->state.getSpace() == b->state.getSpace());
            return a->state.getSpace()->distance(a->state.get(), b->state.get());
        }
    };
    ompl::NearestNeighborsGNATNoThreadSafety<boost::shared_ptr<StateWithId>> m_goal_states;

    ompl::base::PlannerPtr CreatePlanner(OMPLPlannerParameters const& params);

    bool GetParametersCommand(std::ostream& sout, std::istream& sin) const;

    bool ReadStates(std::istream& sin, std::vector<ompl::base::ScopedState<ompl::base::StateSpace>>& states) const;
    bool ReadState(std::istream& sin, ompl::base::ScopedState<ompl::base::StateSpace>& state) const;
    void SetGoals(std::vector<ompl::base::ScopedState<ompl::base::StateSpace>>& states);
};

typedef boost::shared_ptr<RedirectableOMPLPlanner> RedirectableOMPLPlannerPtr;

} // namespace or_ompl

#endif // OR_REDIRECTABLEOMPLPLANNER_H_