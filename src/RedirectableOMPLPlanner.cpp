/***********************************************************************

Copyright (c) 2018, KTH Royal Institute of Technology
All rights reserved.

Authors: Joshua Haustein <haustein@kth.se>

Based on OMPLPlanner.cpp by authors:
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

#include <boost/chrono.hpp>
#include <boost/foreach.hpp>
#include <boost/make_shared.hpp>
#include <boost/scope_exit.hpp>
#include <ompl/base/StateSpace.h>
#include <ompl/base/StateSpaceTypes.h>
#include <ompl/base/goals/GoalStates.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/config.h>
// need to include some planners here to because we need to be able to cast
#include <ompl/geometric/planners/prm/LazyPRM.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <time.h>
#include <tinyxml.h>

#include <or_ompl/OMPLConversions.h>
#include <or_ompl/PlannerRegistry.h>
#include <or_ompl/RedirectableOMPLPlanner.h>
#include <or_ompl/TSRGoal.h>
#include <or_ompl/config.h>

namespace or_ompl {

RedirectableOMPLPlanner::RedirectableOMPLPlanner(OpenRAVE::EnvironmentBasePtr penv,
    PlannerFactory const& planner_factory)
    : OpenRAVE::PlannerBase(penv)
    , m_initialized(false)
    , m_planner_factory(planner_factory)
{

    RegisterCommand("GetParameters",
        boost::bind(&RedirectableOMPLPlanner::GetParametersCommand, this, _1, _2),
        "returns the list of accepted planner parameters");

    RegisterCommand("GetParameterValue",
        boost::bind(&RedirectableOMPLPlanner::GetParameterValCommand, this, _1, _2),
        "returns the value of a specific parameter");

    RegisterCommand("GetTimes",
        boost::bind(&RedirectableOMPLPlanner::GetTimes, this, _1, _2),
        "get timing information from last plan");

    RegisterCommand("GetReachedGoals",
        boost::bind(&RedirectableOMPLPlanner::GetReachedGoals, this, _1, _2),
        "get ids of reached goals in last plan");

    RegisterCommand("IsSupportingGoalReset",
        boost::bind(&RedirectableOMPLPlanner::IsSupportingGoalReset, this, _1, _2),
        "Return whether the underlying planner supports goal resetting.");

    RegisterCommand("ResetGoals",
        boost::bind(&RedirectableOMPLPlanner::ResetGoals, this, _1, _2),
        "Reset motion planning goals to the given goals without resetting planner.");

    RegisterCommand("AddWaypoints",
        boost::bind(&RedirectableOMPLPlanner::AddWaypoints, this, _1, _2),
        "Add collision-free configrations that the planner may use");

    RegisterCommand("GetReachedPoints",
        boost::bind(&RedirectableOMPLPlanner::GetReachedPoints, this, _1, _2),
        "Return all reached configurations");
    m_goal_states.setDistanceFunction(StateWithId::distance);
}

RedirectableOMPLPlanner::~RedirectableOMPLPlanner()
{
}

bool RedirectableOMPLPlanner::InitPlan(OpenRAVE::RobotBasePtr robot, std::istream& input)
{
    OMPLPlannerParametersPtr params = boost::make_shared<OMPLPlannerParameters>();
    input >> *params;
    return InitPlan(robot, params);
}

bool RedirectableOMPLPlanner::InitPlan(OpenRAVE::RobotBasePtr robot,
    PlannerParametersConstPtr params_raw)
{
    m_initialized = false;

    try {
        typedef ompl::base::ScopedState<ompl::base::StateSpace> ScopedState;

        if (!robot) {
            RAVELOG_ERROR("Robot must not be NULL.\n");
            return false;
        } else if (!params_raw) {
            RAVELOG_ERROR("Parameters must not be NULL.\n");
            return false;
        }

        m_robot = robot;
        m_totalPlanningTime = 0.0;

        std::vector<int> dof_indices = robot->GetActiveDOFIndices();
        const unsigned int num_dof = dof_indices.size();
        m_parameters = boost::make_shared<OMPLPlannerParameters>();
        m_parameters->copy(params_raw);

        RAVELOG_DEBUG("Creating state space.\n");
        m_state_space = CreateStateSpace(robot, *m_parameters);
        if (!m_state_space) {
            RAVELOG_ERROR("Failed creating state space.\n");
            return false;
        }

        RAVELOG_DEBUG("Creating OMPL setup.\n");
        m_simple_setup.reset(new ompl::geometric::SimpleSetup(m_state_space));

        RAVELOG_DEBUG("Setting state validity checker.\n");
        if (m_state_space->isCompound()) {
            m_or_validity_checker.reset(new OrStateValidityChecker(
                m_simple_setup->getSpaceInformation(), m_robot, dof_indices, m_parameters->m_doBaked));
        } else {
            m_or_validity_checker.reset(new RealVectorOrStateValidityChecker(
                m_simple_setup->getSpaceInformation(), m_robot, dof_indices, m_parameters->m_doBaked));
        }
#ifdef OR_OMPL_HAS_BOOSTSMARTPTRS
        m_simple_setup->setStateValidityChecker(
            boost::static_pointer_cast<ompl::base::StateValidityChecker>(m_or_validity_checker));
#else
        m_simple_setup->setStateValidityChecker(
            std::static_pointer_cast<ompl::base::StateValidityChecker>(m_or_validity_checker));
#endif

        // start validity checker
        m_or_validity_checker->start();
        BOOST_SCOPE_EXIT((m_or_validity_checker))
        {
            m_or_validity_checker->stop();
        }
        BOOST_SCOPE_EXIT_END

        RAVELOG_DEBUG("Setting initial configuration.\n");
        if (m_parameters->vinitialconfig.size() % num_dof != 0) {
            RAVELOG_ERROR("Start configuration has incorrect DOF;"
                          " expected multiple of %d, got %d.\n",
                num_dof, m_parameters->vinitialconfig.size());
            return false;
        }
        unsigned int num_starts = m_parameters->vinitialconfig.size() / num_dof;
        if (num_starts == 0) {
            RAVELOG_ERROR("No initial configurations provided.\n");
            return false;
        }

        if (num_starts == 1) {
            ScopedState q_start(m_state_space);
            for (size_t i = 0; i < num_dof; i++) {
                q_start[i] = m_parameters->vinitialconfig[i];
            }
            if (!m_or_validity_checker->isValid(q_start.get())) {
                RAVELOG_ERROR("Single initial configuration in collision.\n");
                return false;
            }
        }

        for (unsigned int istart = 0; istart < num_starts; istart++) {
            ScopedState q_start(m_state_space);
            for (size_t i = 0; i < num_dof; i++) {
                q_start[i] = m_parameters->vinitialconfig[istart * num_dof + i];
            }
            m_simple_setup->addStartState(q_start);
        }

        // clear m_goal_states independent of the type of goals
        m_goal_states.clear();
        RAVELOG_DEBUG("Setting goal configuration.\n");
        std::vector<TSRChain::Ptr> goal_chains;
        BOOST_FOREACH (TSRChain::Ptr tsr_chain, m_parameters->m_tsrchains) {
            if (tsr_chain->sampleGoal()) {
                tsr_chain->setEnv(robot->GetEnv()); // required to enable distance to TSR chains
                goal_chains.push_back(tsr_chain);
            } else {
                RAVELOG_ERROR("Only goal TSR chains are supported by OMPL. Failing.\n");
                return false;
            }
        }

        if (goal_chains.size() > 0 && m_parameters->vgoalconfig.size() > 0) {
            RAVELOG_ERROR("A goal TSR chain has been supplied and a goal configuration"
                          " has been specified. The desired behavior is ambiguous."
                          " Please specified one or the other.\n");
            return false;
        }

        if (goal_chains.size() > 0) {
            TSRGoal::Ptr goaltsr(new TSRGoal(m_simple_setup->getSpaceInformation(),
                goal_chains,
                robot,
                m_or_validity_checker));
            m_simple_setup->setGoal(goaltsr);
        } else {
            if (m_parameters->vgoalconfig.size() % num_dof != 0) {
                RAVELOG_ERROR("End configuration has incorrect DOF;"
                              "  expected multiple of %d, got %d.\n",
                    num_dof, m_parameters->vgoalconfig.size());
                return false;
            }
            unsigned int num_goals = m_parameters->vgoalconfig.size() / num_dof;
            if (num_goals == 0) {
                RAVELOG_ERROR("No goal configurations provided.\n");
                return false;
            }

            if (num_goals == 1) {
                ScopedState q_goal(m_state_space);
                for (size_t i = 0; i < num_dof; i++) {
                    q_goal[i] = m_parameters->vgoalconfig[i];
                }

                if (!m_or_validity_checker->isValid(q_goal.get())) {
                    RAVELOG_ERROR("Single goal configuration is in collision.\n");
                    return false;
                }

                m_simple_setup->setGoalState(q_goal);
                m_goal_states.add(boost::make_shared<StateWithId>(q_goal, 0));
            } else {
                // if multiple possible goals specified,
                // don't check them all (this might be expensive)
                // and instead lead the planner check some
                ompl::base::GoalPtr ompl_goals(new ompl::base::GoalStates(
                    m_simple_setup->getSpaceInformation()));
                for (unsigned int igoal = 0; igoal < num_goals; igoal++) {
                    ScopedState q_goal(m_state_space);
                    for (size_t i = 0; i < num_dof; i++) {
                        q_goal[i] = m_parameters->vgoalconfig[igoal * num_dof + i];
                    }
                    ompl_goals->as<ompl::base::GoalStates>()->addState(q_goal);
                    m_goal_states.add(boost::make_shared<StateWithId>(q_goal, igoal));
                }
                m_simple_setup->setGoal(ompl_goals);
            }
        }

        RAVELOG_DEBUG("Creating planner.\n");
        m_planner = CreatePlanner(*m_parameters);
        if (!m_planner) {
            RAVELOG_ERROR("Failed creating OMPL planner.\n");
            return false;
        }
        m_simple_setup->setPlanner(m_planner);

        m_initialized = true;
        return true;
    } catch (std::runtime_error const& e) {
        RAVELOG_ERROR("InitPlan failed: %s\n", e.what());
        return false;
    }
}

ompl::base::PlannerPtr RedirectableOMPLPlanner::CreatePlanner(
    OMPLPlannerParameters const& params)
{
    // Create the planner.
    ompl::base::SpaceInformationPtr const spaceInformation
        = m_simple_setup->getSpaceInformation();

    ompl::base::PlannerPtr planner(m_planner_factory(spaceInformation));
    if (!planner) {
        RAVELOG_ERROR("Failed creating planner.");
        return ompl::base::PlannerPtr();
    }

    // Populate planner parameters from the PlannerParameters.
    std::string const params_str = "<ExtraParams>"
        + m_parameters->_sExtraParameters
        + "</ExtraParams>";
    TiXmlDocument doc_xml;
    doc_xml.Parse(params_str.c_str());
    if (doc_xml.Error()) {
        RAVELOG_ERROR("Failed parsing XML parameters: %s\n",
            doc_xml.ErrorDesc());
        return ompl::base::PlannerPtr();
    }

    TiXmlElement const* root_xml = doc_xml.RootElement();
    std::vector<std::pair<std::string, std::string>> params_vec;

    for (TiXmlElement const* it_ele = root_xml->FirstChildElement();
         it_ele;
         it_ele = it_ele->NextSiblingElement()) {
        // Extract the property name.
        std::string const key = it_ele->ValueStr();

        // Extract the property value.
        TiXmlNode const* node = it_ele->FirstChild();
        if (!node) {
            RAVELOG_ERROR("Failed parsing planner parameters:"
                          " Element '%s' does not contain a value.\n",
                key.c_str());
            return ompl::base::PlannerPtr();
        }
        TiXmlText const* text = node->ToText();
        TiXmlNode const* next_node = node->NextSibling();
        if (!text || next_node) {
            RAVELOG_ERROR("Failed parsing planner parameters:"
                          " Element '%s' contains complex data.\n",
                key.c_str());
        }
        std::string const value = text->Value();

        params_vec.push_back(std::make_pair(key, value));
    }

    ompl::base::ParamSet& param_set = planner->params();
    bool is_success = true;
    for (std::vector<std::pair<std::string, std::string>>::iterator
             it
         = params_vec.begin();
         it != params_vec.end(); it++) {
        is_success = param_set.setParam(it->first, it->second);
        if (!is_success)
            break;
    }

    // Print out the list of valid parameters.
    if (!is_success) {
        std::vector<std::string> param_names;
        param_set.getParamNames(param_names);

        std::stringstream param_stream;
        BOOST_FOREACH (std::string const& param_name, param_names) {
            param_stream << " " << param_name;
        }
        std::string const param_str = param_stream.str();

        RAVELOG_ERROR("Invalid planner parameters."
                      " The following parameters are supported:%s\n",
            param_str.c_str());
        return ompl::base::PlannerPtr();
    }
    return planner;
}

OpenRAVE::PlannerStatus RedirectableOMPLPlanner::PlanPath(OpenRAVE::TrajectoryBasePtr ptraj)
{
    if (!m_initialized) {
        RAVELOG_ERROR("Unable to plan. Did you call InitPlan?\n");
        return OpenRAVE::PS_Failed;
    }

    boost::chrono::steady_clock::time_point const tic
        = boost::chrono::steady_clock::now();

    OpenRAVE::PlannerStatus planner_status;
    try {
        // TODO: Configure anytime algorithms to keep planning.
        //m_simpleSetup->getGoal()->setMaximumPathLength(0.0);

        // start validity checker
        m_or_validity_checker->start();
        BOOST_SCOPE_EXIT((m_or_validity_checker))
        {
            m_or_validity_checker->stop();
        }
        BOOST_SCOPE_EXIT_END

        ompl::base::PlannerStatus ompl_status;
        RAVELOG_DEBUG_FORMAT("Planner timeout is %f", m_parameters->m_timeLimit);
        ompl_status = m_simple_setup->solve(m_parameters->m_timeLimit);

        // Handle OMPL return codes, set planner_status and ptraj
        if (ompl_status == ompl::base::PlannerStatus::EXACT_SOLUTION
            || ompl_status == ompl::base::PlannerStatus::APPROXIMATE_SOLUTION) {

            if (m_simple_setup->haveExactSolutionPath()) {
                ToORTrajectory(m_robot, m_simple_setup->getSolutionPath(), ptraj);
                if (ompl_status == ompl::base::PlannerStatus::EXACT_SOLUTION) {
                    planner_status = OpenRAVE::PS_HasSolution;
                } else {
                    planner_status = OpenRAVE::PS_InterruptedWithSolution;
                }
            } else {
                RAVELOG_ERROR("Planner returned %s, but no path found!\n", ompl_status.asString().c_str());
                planner_status = OpenRAVE::PS_Failed;
            }

        } else {
            // Intended to handle:
            // - PlannerStatus::INVALID_START
            // - PlannerStatus::INVALID_GOAL
            // - PlannerStatus::UNRECOGNIZED_GOAL_TYPE
            // - PlannerStatus::CRASH
            // - PlannerStatus::ABORT
            // - PlannerStatus::TIMEOUT
            // (these cases are not handled explicitly because different versions
            //  of OMPL support different error cases)
            RAVELOG_ERROR("Planner returned %s.\n", ompl_status.asString().c_str());
            planner_status = OpenRAVE::PS_Failed;
        }

    } catch (std::runtime_error const& e) {
        RAVELOG_ERROR("Planning failed: %s\n", e.what());
        planner_status = OpenRAVE::PS_Failed;
    }

    boost::chrono::steady_clock::time_point const toc
        = boost::chrono::steady_clock::now();
    m_totalPlanningTime += boost::chrono::duration_cast<
        boost::chrono::duration<double>>(toc - tic)
                               .count();

    return planner_status;
}

bool RedirectableOMPLPlanner::GetParametersCommand(std::ostream& sout, std::istream& sin) const
{
    typedef std::map<std::string, ompl::base::GenericParamPtr> ParamMap;

    ompl::base::PlannerPtr planner;
    if (m_planner) {
        planner = m_planner;
    }
    // We need an instance of the planner to query its ParamSet. Unfortunately,
    // constructing the planner requires a SpaceInformationPtr, which can only
    // be generated from an existing StateSpace. As a workaround, we construct
    // a simple one-DOF state space and make a temporary planner instance.
    else {
        ompl::base::StateSpacePtr const state_space(
            new ompl::base::RealVectorStateSpace(1));
        ompl::base::SpaceInformationPtr const space_information(
            new ompl::base::SpaceInformation(state_space));
        planner.reset(m_planner_factory(space_information));
    }

    // Query the supported parameters. Each planner has a name and a "range
    // suggestion", which is used to generate the GUI in OMPL.app.
    ompl::base::ParamSet const& param_set = planner->params();
    ParamMap const& param_map = param_set.getParams();

    ParamMap::const_iterator it;
    for (it = param_map.begin(); it != param_map.end(); ++it) {
        sout << it->first << " (" << it->second->getRangeSuggestion() << ")\n";
    }

    return true;
}

bool RedirectableOMPLPlanner::GetParameterValCommand(std::ostream& sout, std::istream& sin) const
{
    typedef std::map<std::string, ompl::base::GenericParamPtr> ParamMap;
    //Obtain argument from input stream
    std::string inp_arg;
    sin >> inp_arg;

    ompl::base::PlannerPtr planner;
    if (m_planner) {
        planner = m_planner;
    }
    // We need an instance of the planner to query its ParamSet. Unfortunately,
    // constructing the planner requires a SpaceInformationPtr, which can only
    // be generated from an existing StateSpace. As a workaround, we construct
    // a simple one-DOF state space and make a temporary planner instance.
    else {
        ompl::base::StateSpacePtr const state_space(
            new ompl::base::RealVectorStateSpace(1));
        ompl::base::SpaceInformationPtr const space_information(
            new ompl::base::SpaceInformation(state_space));
        planner.reset(m_planner_factory(space_information));
    }

    // Query the supported parameters. Each planner has a name and a "range
    // suggestion", which is used to generate the GUI in OMPL.app.

    ompl::base::ParamSet const& param_set = planner->params();
    std::string value;

    //Check if in parameter map
    bool in_map = param_set.getParam(inp_arg, value);

    if (!in_map) {
        RAVELOG_ERROR("Parameter not in set\n");
        throw OpenRAVE::openrave_exception("Parameter not in set", OpenRAVE::ORE_InvalidArguments);
    } else {
        //Output key-value pair
        sout << inp_arg << " " << value;
    }

    return true;
}

bool RedirectableOMPLPlanner::GetTimes(std::ostream& sout, std::istream& sin) const
{
    if (!m_or_validity_checker) {
        RAVELOG_ERROR("GetTimes cannot be called before a plan has been initialized!\n");
        return false;
    }
    sout << "checktime " << m_or_validity_checker->getTotalCollisionTime();
    sout << " totaltime " << m_totalPlanningTime;
    sout << " n_checks " << m_or_validity_checker->getNumCollisionChecks();
    return true;
}

bool RedirectableOMPLPlanner::GetReachedGoals(std::ostream& sout, std::istream& sin) const
{
    RAVELOG_DEBUG("Called GetReachedGoals");
    typedef ompl::base::ScopedState<ompl::base::StateSpace> ScopedState;
    if (!m_initialized) {
        RAVELOG_ERROR("GetReachedGoals cannot be called before a plan has been initialized!\n");
        return false;
    }
    int reached_goal_id = -1;
    if (m_simple_setup->haveSolutionPath()) {
        auto last_solution = m_simple_setup->getSolutionPath();
        auto end_state = last_solution.getState(last_solution.getStateCount() - 1);
        auto query_state = boost::make_shared<StateWithId>(ScopedState(m_state_space), 0);
        m_state_space->copyState(query_state->state.get(), end_state);
        auto closest_input_goal = m_goal_states.nearest(query_state);
        assert(m_state_space->distance(closest_input_goal->state.get(), query_state->state.get()) == 0.0);
        reached_goal_id = closest_input_goal->id;
    }
    sout << reached_goal_id;
    return true;
}

bool RedirectableOMPLPlanner::IsSupportingGoalReset(std::ostream& sout, std::istream& sin) const
{
    if (!m_initialized) {
        RAVELOG_ERROR("ResetGoals cannot be called before a plan has been initialized!\n");
        return false;
    }

    auto planner_name = m_planner->getName();
    bool supports_goal_reset = planner_name == "LazyPRM" or planner_name == "PRM"
        or planner_name == "LazyPRMstar" or planner_name == "PRMstar";
    sout << supports_goal_reset;
    return true;
}

bool RedirectableOMPLPlanner::ResetGoals(std::ostream& sout, std::istream& sin)
{
    typedef ompl::base::ScopedState<ompl::base::StateSpace> ScopedState;
    if (!m_initialized) {
        RAVELOG_ERROR("ResetGoals cannot be called before a plan has been initialized!\n");
        return false;
    }
    RAVELOG_DEBUG("Called ResetGoals");
    std::vector<ScopedState> states;
    bool read_success = ReadStates(sin, states);
    if (!read_success)
        return false;
    // set new goals
    auto planner = m_simple_setup->getPlanner();
    { // LazyPRM and LazyPRM*
        auto lazy_prm_planner = std::dynamic_pointer_cast<ompl::geometric::LazyPRM>(planner);
        if (lazy_prm_planner) {
            lazy_prm_planner->clearQuery();
            SetGoals(states);
            return true;
        }
    }
    { // PRM and PRM*
        auto prm_planner = std::dynamic_pointer_cast<ompl::geometric::PRM>(planner);
        if (prm_planner) {
            prm_planner->clearQuery();
            SetGoals(states);
            return true;
        }
    }
    RAVELOG_ERROR("Can not reset goals for the set planner type." + planner->getName());
    return false;
}

bool RedirectableOMPLPlanner::AddWaypoints(std::ostream& sout, std::istream& sin)
{
    if (!m_initialized) {
        RAVELOG_ERROR("AddWaypoints cannot be called before a plan has been initialized!\n");
        return false;
    }
    RAVELOG_ERROR("Called AddWaypoints: NOT IMPLEMENTED");
    return true;
}

bool RedirectableOMPLPlanner::GetReachedPoints(std::ostream& sout, std::istream& sin) const
{
    if (!m_initialized) {
        RAVELOG_ERROR("GetReachedPoints cannot be called before a plan has been initialized!\n");
        return false;
    }
    RAVELOG_ERROR("Called GetReachedPoints: NOT IMPLEMENTED");
    return true;
}

bool RedirectableOMPLPlanner::ReadStates(std::istream& sin, std::vector<ompl::base::ScopedState<ompl::base::StateSpace>>& states) const
{
    typedef ompl::base::ScopedState<ompl::base::StateSpace> ScopedState;
    char c = '\0';
    if (!sin.good())
        return false;
    sin >> c; // '[ remove outer bracket for multiple states
    if (c != '[') {
        RAVELOG_ERROR("Failed to read states from stream. Invalid format!");
        return false;
    }
    ScopedState new_state(m_state_space);
    bool all_good = true;
    while (all_good) {
        all_good = ReadState(sin, new_state);
        if (all_good) {
            states.push_back(new_state);
            sin >> c;
            all_good = c == ',';
        }
    }
    return all_good;
}

bool RedirectableOMPLPlanner::ReadState(std::istream& sin, ompl::base::ScopedState<ompl::base::StateSpace>& state) const
{
    char c = '\0';
    if (!sin.good())
        return false;
    sin >> c;
    if (c != '[') {
        RAVELOG_ERROR("Failed to read states from stream. Invalid format!");
        return false;
    }
    int idx = 0;
    while (sin.good() and c != ']' and idx < m_state_space->getDimension()) {
        float val;
        sin >> val;
        state[idx] = val;
        sin >> c; // ','
        ++idx;
    }
    if (c != ']') {
        RAVELOG_ERROR("Failed to read states from stream. Invalid format!");
        return false;
    }
    return true;
}

void RedirectableOMPLPlanner::SetGoals(std::vector<ompl::base::ScopedState<ompl::base::StateSpace>>& states)
{

    std::shared_ptr<ompl::base::GoalStates> ompl_goals = std::dynamic_pointer_cast<ompl::base::GoalStates>(m_simple_setup->getGoal());
    if (!ompl_goals) { // we didn't have a multi-goal before
        ompl_goals = std::make_shared<ompl::base::GoalStates>(m_simple_setup->getSpaceInformation());
        m_simple_setup->setGoal(ompl_goals);
    }
    // remove old goals
    ompl_goals->clear();
    // add new goals
    for (unsigned int igoal = 0; igoal < states.size(); igoal++) {
        ompl_goals->addState(states[igoal]);
        m_goal_states.add(boost::make_shared<StateWithId>(states[igoal], igoal));
    }
    m_simple_setup->setGoal(ompl_goals);
}

} // namespace or_ompl