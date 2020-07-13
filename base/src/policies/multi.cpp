/** \file multi.cpp
 * \brief Combining policy source file. 
 *
 * \author    Wouter Caarls <wouter@caarls.org>
 * \date      2018-03-13
 *
 * \copyright \verbatim
 * Copyright (c) 2018, Wouter Caarls
 * All rights reserved.
 *
 * This file is part of GRL, the Generic Reinforcement Learning library.
 *
 * GRL is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * \endverbatim
 */

#include <grl/policies/multi.h>
#include <queue>

using namespace grl;

REGISTER_CONFIGURABLE(MultiPolicy)

void MultiPolicy::request(ConfigurationRequest *config)
{
  config->push_back(CRP("strategy", "Combination strategy", strategy_str_, CRP::Configuration, {"binning", "density_based", "data_center", "data_center_mean_mov", "mean", "meanmov", "random", "static", "value_based"}));
  config->push_back(CRP("sampler", "sampler", "Sampler for value-based strategy", sampler_, true));
  config->push_back(CRP("bins", "Binning Simple Discretization", bins_));
  config->push_back(CRP("static_policy", "Static Policy Chosen to Learning", static_policy_));
  config->push_back(CRP("r_distance_parameter", "R Distance Parameter", r_distance_parameter_));

  config->push_back(CRP("policy", "mapping/policy", "Sub-policies", &policy_));
  config->push_back(CRP("value", "mapping", "Values of sub-policy actions", &value_, true));
  
  config->push_back(CRP("output_min", "vector.action_min", "Lower limit on outputs", min_, CRP::System));
  config->push_back(CRP("output_max", "vector.action_max", "Upper limit on outputs", max_, CRP::System));

  config->push_back(CRP("actions", "signal/vector.action", "Suggested actions", CRP::Provided));
}

void MultiPolicy::configure(Configuration &config)
{
  strategy_str_ = config["strategy"].str();
  if (strategy_str_ == "binning")
    strategy_ = csBinning;
  else if (strategy_str_ == "density_based")
    strategy_ = csDensityBased;
  else if (strategy_str_ == "data_center")
    strategy_ = csDataCenter;
  else if (strategy_str_ == "data_center_mean_mov")
    strategy_ = csDataCenterMeanMov;
  else if (strategy_str_ == "mean")
    strategy_ = csMean;
  else if (strategy_str_ == "mean_mov")
    strategy_ = csMeanMov;
  else if (strategy_str_ == "random")
    strategy_ = csRandom;
  else if (strategy_str_ == "static")
    strategy_ = csStatic;
  else if (strategy_str_ == "value_based")
    strategy_ = csValueBased;
  else
    throw bad_param("mapping/policy/multi:strategy");

  sampler_ = (Sampler*)config["sampler"].ptr();

  bins_ = config["bins"];

  static_policy_ = config["static_policy"];
  if (static_policy_ >= 0 && static_policy_ < policy_.size())
    throw bad_param("policy/static_policy:static_policy is out of bound");

  r_distance_parameter_ = config["r_distance_parameter"];

  min_ = config["output_min"].v();
  max_ = config["output_max"].v();
  if (min_.size() != max_.size() || !min_.size())
    throw bad_param("policy/action:{output_min,output_max}");

  policy_ = *(ConfigurableList*)config["policy"].ptr();
  value_ = *(ConfigurableList*)config["value"].ptr();
  
  if (policy_.empty())
    throw bad_param("mapping/policy/multi:policy is empty");

  mean_mov_ = new std::vector<double>(policy_.size());
  
  action_ = new VectorSignal();  
  config.set("actions", action_);

  *pt_iterations_ = 0;
}

void MultiPolicy::reconfigure(const Configuration &config)
{
}

void MultiPolicy::act(const Observation &in, Action *out) const
{
  Action tmp_action;
  LargeVector dist;
  Vector dummy;
  std::vector<size_t> ii_max_density;
  int n_policies = policy_.size();
  std::vector<Action> actions_actors(n_policies);
  policy_[0]->act(in, &actions_actors[0]);
  int n_dimension = actions_actors[0].v.size();
  Vector send_actions(n_policies);
//  double* result_np = new double[n_policies];
  double* result_nd = new double[n_dimension];
  size_t ii;
  
  switch (strategy_)
  {
    case csBinning:
    {
      double* actions_actors = new double[policy_.size()];
      
      CRAWL("MultiPolicy::csBinning histogram");

      double* histogram = new double[bins_];
      for (size_t ii=0; ii < bins_; ++ii)
          histogram[ii] = 0;

      for (size_t jj=0; jj < n_dimension; ++jj)
      {
        CRAWL("MultiPolicy:: min_["<< jj <<"]): " << min_[jj] << ", max_["<< jj <<"]): " << max_[jj] << "\n");
        for (size_t ii=0; ii != policy_.size(); ++ii)
        {
          policy_[ii]->act(in, &tmp_action);
          actions_actors[ii] = tmp_action.v[jj];
          histogram[std::min((int)floor(bins_ * (actions_actors[ii] - min_[jj]) / (max_[jj] - min_[jj]) ), bins_-1)]++;
          CRAWL("MultiPolicy::(*a["<< ii <<"]["<< jj <<"]): " << actions_actors[ii] << "\n");
        }

        size_t cnt_binmax = 0;
        size_t binmax = 0;
        for (size_t ii=0; ii < bins_; ++ii)
        {
          if (cnt_binmax < histogram[ii])
          {
            cnt_binmax = histogram[ii];
            binmax = ii;
            CRAWL("MultiPolicy::binmax: " << binmax << " - cnt_binmax:" << cnt_binmax << "\n");
          }
          CRAWL("MultiPolicy::histogram[" << ii << "]: " << histogram[ii] << "\n");
        }
        CRAWL("MultiPolicy::binmax: " << binmax << "\n");

        size_t n_binmax = 0;
        size_t i_bin = 0;
        result_nd[jj] = 0.0;
        for (size_t ii=0; ii < policy_.size(); ++ii)
        {
          i_bin = std::min((int)floor(bins_ * (actions_actors[ii] - min_[jj]) / (max_[jj] - min_[jj]) ), bins_-1);
          CRAWL("MultiPolicy::i_bin: " << i_bin << "\n");
          CRAWL("MultiPolicy::histogram[binmax]: " << histogram[binmax] << " - histogram[i_bin]: " << histogram[i_bin] << "\n");
          if ( histogram[binmax] == histogram[i_bin] )
          {
            result_nd[jj] += actions_actors[ii];
            CRAWL("MultiPolicy::a[" << ii << "]: " << actions_actors[ii] << "\n");
            ++n_binmax;
          }
        }
        result_nd[jj] = result_nd[jj] / n_binmax;
        CRAWL("MultiPolicy::result_nd: " << result_nd[jj] << " n_binmax: " << n_binmax << "\n");
      }
      dist = ConstantLargeVector(n_dimension, *result_nd);
    }
    break;
        
    case csDensityBased:
    {
      std::string strspace = "";
      for(size_t kk=0; kk < 32; ++kk)
        strspace += " ";

      std::vector<Action> aa_normalized(n_policies);
      std::vector<double> density(n_policies);
      
      std::vector<Action>::iterator it_norm = aa_normalized.begin();
      ii = 0;
      for(std::vector<Action>::iterator it = actions_actors.begin(); it != actions_actors.end(); ++it, ++ii)
      {
        policy_[ii]->act(in, &*it);
        tmp_action.v = -1 + 2*( ((*it).v - min_) / (max_ - min_) );
        (*it_norm) = tmp_action;
        ++it_norm;
        CRAWL("ii: " << ii << " actions_actors: " << (*it).v << " normalized: " << tmp_action.v);

        send_actions[ii] = it->v[0];
      }

      action_->set(send_actions);

      double max = -1 * std::numeric_limits<double>::infinity();
      std::vector<Action>::iterator it, it2, i_max;
      ii = 0;
      for( it=aa_normalized.begin(); it != aa_normalized.end(); ++it, ++ii)
      {
        std::string strtmp = "";
        double r_dist = 0.0;
        for( it2=aa_normalized.begin(); it2 != aa_normalized.end(); ++it2)
        {
          double expoent = 0.0;
          for(size_t jj = 0; jj != n_dimension; ++jj)
          {
            expoent += pow((*it).v[jj] - (*it2).v[jj], 2);
            //strtmp += strspace + "expoent(jj:" + std::to_string(jj) + "): " +  std::to_string(expoent) + " (1): " + std::to_string((*it).v[jj]) + " (2): " + std::to_string((*it2).v[jj]) + "\n";
          }
          //strtmp += strspace + strspace + "r_dist(before): " + std::to_string(r_dist) + " expoent: " + std::to_string((-1 * expoent / pow(r_distance_parameter_, 2))) + " exp: " + std::to_string(exp( -1 * expoent / pow(r_distance_parameter_, 2))) + "\n";
          r_dist = r_dist + exp( -1 * expoent / pow(r_distance_parameter_, 2) );
        }
        density.push_back( r_dist );
        //CRAWL(strtmp << strspace << "sum(expo) - density[ii:" << ii << "]: " << r_dist );
        if (r_dist > max)
        {
          max = r_dist;
          i_max = it;
          ii_max_density.clear();
          ii_max_density.push_back(ii);
        } else if (r_dist == max)
          ii_max_density.push_back(ii);
        CRAWL("******************************************************************max(ii:"<<ii<<") = " << max );
      }

      for (size_t jj=0; jj < n_dimension; ++jj)
      {
        int aleatorio = rand();
        size_t index = ii_max_density.at(aleatorio%ii_max_density.size());
        CRAWL( "MultiPolicy::ii_max_density.size(): " << ii_max_density.size() << " aleatorio: " << aleatorio << " index: " << index);
        result_nd[jj] = actions_actors[index].v[jj];
        //CRAWL( "MultiPolicy::result_[ii_max=" << index << "][jj:" << jj << "]: " << result[jj] );
      }

      dist = ConstantLargeVector( n_dimension, *result_nd );
    }
    break;
        
    case csDataCenter:
    {
      CRAWL("MultiPolicy::csDataCenter::starting");
      std::deque<Action> actions_actors2(policy_.size());
      LargeVector mean;
      bool first = true;
      ii = 0;
      
      for(std::deque<Action>::iterator it = actions_actors2.begin(); it != actions_actors2.end(); ++it, ++ii)
      {
        policy_[ii]->act(in, &*it);
        if (first)
          mean = it->v;
        else
          mean = mean + it->v;
        CRAWL("MultiPolicy::csDataCenter::collecting actions_actors[ii:" << ii << "]->v: " << it->v);
        first = false;
      }
      mean = mean / actions_actors2.size();
      CRAWL("MultiPolicy::csDataCenter::collecting mean: " << mean);
      
      while(actions_actors2.size() > bins_)
      {
        //PRINTLN
        //for (std::deque <Action> :: iterator it = actions_actors2.begin(); it != actions_actors2.end(); ++it)
        //  for (size_t ii = 0; ii < (*it).size(); ++ii)
        //    CRAWL( '\t' << (*it)[ii]);
        //CRAWL('\n');
        
        //EUCLIDIAN DISTANCE
        double max = 0;
        std::deque <Action> :: iterator i_max, it;
        ii = 0;
        size_t ii_max = 0;
        for( it=actions_actors2.begin(); it < actions_actors2.end(); ++it, ++ii)
        { 
          double dist = sum(pow(actions_actors2.at(ii).v - mean, 2));
          CRAWL("MultiPolicy::csDataCenter::euclidian distance:dist: " << dist);
          if (dist > max)
          {
            max = dist;
            i_max = it;
            ii_max = ii;
            ii_max_density.clear();
            ii_max_density.push_back(ii);
            CRAWL("MultiPolicy::csDataCenter::euclidian distance:max: " << max << " ii_max: " << ii_max);
          } else if (dist == max)
            ii_max_density.push_back(ii);
        }

        int aleatorio = rand();
        size_t index = ii_max_density.at(aleatorio%ii_max_density.size());
        CRAWL("MultiPolicy::ii_max_density.size(): " << ii_max_density.size() << " aleatorio: " << aleatorio << " index: " << index);
          
        CRAWL("MultiPolicy::csDataCenter::remove outlier");
        //retirando apenas o elemento que está no index i_max
        actions_actors2.erase(actions_actors2.begin()+index);

        //PRINTLN
        for(size_t ii=0; ii < actions_actors2.size(); ++ii)
          CRAWL("MultiPolicy::after remove the i_max actions_actors2: " << actions_actors2[ii]);

        for (size_t ii=0; ii < mean.size(); ++ii)
          mean[ii] = 0;
        
        CRAWL("MultiPolicy::csDataCenter::starting new mean");
      
        bool first = true;
        std::vector<size_t> ii_max_density;
        for(it = actions_actors2.begin(); it < actions_actors2.end(); ++it)
        {
          if (first)
            mean = it->v;
          else
            mean = mean + it->v;
          first = false;
          CRAWL("MultiPolicy::csDataCenter::actions_actors2.v: " << it->v << " mean: " << mean  << "\n");
        }
        mean = mean / actions_actors2.size();
        CRAWL("MultiPolicy::csDataCenter::(mean / actions_actors2.size()): " << mean << "\n");
      }

      dist = mean;
      //actions_actors2[(rand()%2)];
    }
    break;
        
    case csDataCenterMeanMov:
    {
      CRAWL("MultiPolicy::csDataCenterMeanMov::starting");
      std::deque<Action> actions_actors2(policy_.size());
      std::deque<Action>::iterator it;
      std::vector<double>::iterator itd;
      std::vector<size_t> v_id(0);
     
      LargeVector mean;
      bool first = true;
      ii = 0;
      
      for(it = actions_actors2.begin(); it != actions_actors2.end(); ++it, ++ii)
      {
        policy_[ii]->act(in, &*it);
        if (first)
          mean = it->v;
        else
          mean = mean + it->v;
        CRAWL("MultiPolicy::csDataCenter::collecting actions_actors[ii:" << ii << "]->v: " << it->v);
        first = false;
      }
      mean = mean / n_policies;
      CRAWL("MultiPolicy::csDataCenter::collecting mean: " << mean);

      //#############################################################
      //SAVE BETWEEN FIRST AND THIRD QUANTILE
      
      //EUCLIDIAN DISTANCE
      double euclidian_dist = 0;
      ii = 0;
//      size_t ii_max = 0;
      for( it=actions_actors2.begin(); it < actions_actors2.end(); ++it, ++ii)
      {
	      euclidian_dist = sum(pow((*it).v - mean, 2));
	      mean_mov_->at(ii) = (r_distance_parameter_)*euclidian_dist + (1-r_distance_parameter_)*mean_mov_->at(ii);
        CRAWL("MultiPolicy::csMeanMov::a(ii= " << ii << "): "<<(*it).v<<" euclidian distance:dist: " << euclidian_dist);
      }

      std::vector<double> quantile(0);
      for (std::vector<double>::iterator it=mean_mov_->begin(); it!=mean_mov_->end(); ++it)
        quantile.push_back(*it);
      
      if (*pt_iterations_ > bins_)
      {
        std::sort(quantile.begin(), quantile.end());
        size_t size = quantile.size();
//        size_t mid = size/2;
    
        double fst;
        double trd;

        size_t side_length = (size_t) size/2;

        fst = quantile[(size_t) side_length/2];
        trd = quantile[side_length + (size_t)side_length/2];
        
        for(size_t jj = 0; jj < quantile.size(); ++jj)
          CRAWL("MultiPolicy::quantile[jj:"<< jj <<"]" << quantile[jj]);
        
        double q = 0.25;
        size_t n  = quantile.size();
        double id = (n - 1) * q;
        size_t lo = floor(id);
        size_t hi = ceil(id);
        double qs = quantile[lo];
        double h  = (id - lo);
        fst = (1.0 - h) * qs + h * quantile[hi];

        q = 0.75;
        n  = quantile.size();
        id = (n - 1) * q;
        lo = floor(id);
        hi = ceil(id);
        qs = quantile[lo];
        h  = (id - lo);

        trd = (1.0 - h) * qs + h * quantile[hi];
        
        CRAWL("MultiPolicy::fst:" << fst << " ((size_t) side_length/2):"  << ((size_t) side_length/2) << " quantile(fst):" << quantile[fst]);
        CRAWL("MultiPolicy::trd:" << trd << " side_length + (size_t)side_length/2: " << (side_length + (size_t)side_length/2) << " quantile(trd):" << quantile[trd]);

        ii = 0;
        mean = 0;
        for(it = actions_actors2.begin(), itd = mean_mov_->begin(); it != actions_actors2.end(); ++it, ++itd, ++ii)
        {
          if ( (*itd >= fst) && (*itd <= trd))
          {
            v_id.push_back(ii);
            mean = mean + it->v;
          }
        }
        mean = mean / ii;

        size_t size_v_id = v_id.size();
        for(ii=0; ii < size_v_id; ++ii)
          actions_actors2.erase(actions_actors2.begin()+v_id[ii]-ii);

        for(itd = mean_mov_->begin(); itd < mean_mov_->end(); ++itd, ++ii)
          CRAWL("MultiPolicy::("<< *pt_iterations_ << ")csMeanMov[ii:" << ii << "]: " << (*itd));
      } //if (*pt_iterations_ > bins_)
      //############################################################
      
      while(actions_actors2.size() > bins_)
      {
        //PRINTLN
        //for (std::deque <Action> :: iterator it = actions_actors2.begin(); it != actions_actors2.end(); ++it)
        //  for (size_t ii = 0; ii < (*it).size(); ++ii)
        //    CRAWL( '\t' << (*it)[ii]);
        //CRAWL('\n');
        
        //EUCLIDIAN DISTANCE
        double max = 0;
        std::deque <Action> :: iterator i_max, it;
        ii = 0;
        size_t ii_max = 0;
        for( it=actions_actors2.begin(); it < actions_actors2.end(); ++it, ++ii)
        { 
          double dist = sum(pow(actions_actors2.at(ii).v - mean, 2));
          CRAWL("MultiPolicy::csDataCenter::euclidian distance:dist: " << dist);
          if (dist > max)
          {
            max = dist;
            i_max = it;
            ii_max = ii;
            ii_max_density.clear();
            ii_max_density.push_back(ii);
            CRAWL("MultiPolicy::csDataCenter::euclidian distance:max: " << max << " ii_max: " << ii_max);
          } else if (dist == max)
            ii_max_density.push_back(ii);
        }

        int aleatorio = rand();
        size_t index = ii_max_density.at(aleatorio%ii_max_density.size());
        CRAWL("MultiPolicy::ii_max_density.size(): " << ii_max_density.size() << " aleatorio: " << aleatorio << " index: " << index);
          
        CRAWL("MultiPolicy::csDataCenter::remove outlier");
        //retirando apenas o elemento que está no index i_max
        actions_actors2.erase(actions_actors2.begin()+index);

        //PRINTLN
        for(size_t ii=0; ii < actions_actors2.size(); ++ii)
          CRAWL("MultiPolicy::after remove the i_max actions_actors2: " << actions_actors2[ii]);

        for (size_t ii=0; ii < mean.size(); ++ii)
          mean[ii] = 0;
        
        CRAWL("MultiPolicy::csDataCenter::starting new mean");
      
        bool first = true;
        std::vector<size_t> ii_max_density;
        for(it = actions_actors2.begin(); it < actions_actors2.end(); ++it)
        {
          if (first)
            mean = it->v;
          else
            mean = mean + it->v;
          first = false;
          CRAWL("MultiPolicy::csDataCenter::actions_actors2.v: " << it->v << " mean: " << mean  << "\n");
        }
        mean = mean / actions_actors2.size();
        CRAWL("MultiPolicy::csDataCenter::(mean / actions_actors2.size()): " << mean << "\n");
      }

      dist = mean;
      //actions_actors2[(rand()%2)];
    }
    break;
        
    case csMean:
    {
      LargeVector mean;
      bool first = true;
      ii = 0;
      for(std::vector<Action>::iterator it = actions_actors.begin(); it != actions_actors.end(); ++it, ++ii)
      {
        policy_[ii]->act(in, &*it);
        if (first)
          mean = it->v;
        else
          mean = mean + it->v;
        first = false;
      }
      dist = mean / n_policies;
    }
    break;

    case csMeanMov:
    {
      LargeVector mean;
      std::vector<Action>::iterator it;
      std::vector<double>::iterator itd;
      bool first = true;
      ii = 0;
      for(std::vector<Action>::iterator it = actions_actors.begin(); it != actions_actors.end(); ++it, ++ii)
      {
        policy_[ii]->act(in, &*it);
        if (first)
          mean = it->v;
        else
          mean = mean + it->v;
        first = false;
      }
      mean = mean / n_policies;
      
      //EUCLIDIAN DISTANCE
      double euclidian_dist = 0;
      ii = 0;
//      size_t ii_max = 0;
      for( it=actions_actors.begin(); it < actions_actors.end(); ++it, ++ii)
      {
	      euclidian_dist = sum(pow((*it).v - mean, 2));
	      mean_mov_->at(ii) = (r_distance_parameter_)*euclidian_dist + (1-r_distance_parameter_)*mean_mov_->at(ii);
        CRAWL("MultiPolicy::csMeanMov::a(ii= " << ii << "): "<<(*it).v<<" euclidian distance:dist: " << euclidian_dist);
      }

      std::vector<double> quantile(0);
      for (std::vector<double>::iterator it=mean_mov_->begin(); it!=mean_mov_->end(); ++it)
        quantile.push_back(*it);
      
      if (*pt_iterations_ > bins_)
      {
        std::sort(quantile.begin(), quantile.end());
        size_t size = quantile.size();
//        size_t mid = size/2;
    
        double fst;
        double trd;

        size_t side_length = (size_t) size/2;

        fst = quantile[(size_t) side_length/2];
        trd = quantile[side_length + (size_t)side_length/2];
        
        for(size_t jj = 0; jj < quantile.size(); ++jj)
          CRAWL("MultiPolicy::quantile[jj:"<< jj <<"]" << quantile[jj]);
        
        double q = 0.25;
        size_t n  = quantile.size();
        double id = (n - 1) * q;
        size_t lo = floor(id);
        size_t hi = ceil(id);
        double qs = quantile[lo];
        double h  = (id - lo);
        fst = (1.0 - h) * qs + h * quantile[hi];

        q = 0.75;
        n  = quantile.size();
        id = (n - 1) * q;
        lo = floor(id);
        hi = ceil(id);
        qs = quantile[lo];
        h  = (id - lo);

        trd = (1.0 - h) * qs + h * quantile[hi];
        
        CRAWL("MultiPolicy::fst:" << fst << " ((size_t) side_length/2):"  << ((size_t) side_length/2) << " quantile(fst):" << quantile[fst]);
        CRAWL("MultiPolicy::trd:" << trd << " side_length + (size_t)side_length/2: " << (side_length + (size_t)side_length/2) << " quantile(trd):" << quantile[trd]);

        ii = 0;
        mean = 0;
        for(it = actions_actors.begin(), itd = mean_mov_->begin(); it != actions_actors.end(); ++it, ++itd, ++ii)
        {
          if ( (*itd >= fst) && (*itd <= trd))
            mean = mean + it->v;
        }
        mean = mean / ii;

        for(itd = mean_mov_->begin(); itd < mean_mov_->end(); ++itd, ++ii)
          CRAWL("MultiPolicy::("<< *pt_iterations_ << ")csMeanMov[ii:" << ii << "]: " << (*itd));
      } //if (*pt_iterations_ > bins_)

	    *pt_iterations_ = *pt_iterations_ + 1;
      dist = mean;
      
    }
    break;

    case csRandom:
    {
      int aleatorio = rand();
      size_t policy_random = aleatorio%n_policies;
      policy_[policy_random]->act(in, &tmp_action);
      dist = tmp_action.v;

      CRAWL("MultiPolicy::case csRandom policy_[policy_random: " << policy_random << "]->v: " << dist);
    }
    break;
    
    case csStatic:
    {
      size_t policy_chosen = static_policy_%n_policies;
      policy_[policy_chosen]->act(in, &tmp_action);
      dist = tmp_action.v;
      CRAWL("MultiPolicy::case csStatic policy_[policy_chosen:" << policy_chosen << "]->v: " << dist);
    }
    break;

    case csValueBased:
    {
      ii = 0;
      //double* values = new double[n_policies];
      dist = LargeVector::Zero(n_dimension);
      LargeVector values = LargeVector::Zero(n_policies);

      //#ifdef _OPENMP
      //#pragma omp parallel for
      //#endif
      //for(std::vector<Action>::iterator it = actions_actors.begin(); it != actions_actors.end(); ++it, ++ii)
      for(size_t ii=0; ii < actions_actors.size(); ++ii)
      {
        policy_[ii]->act(in, &actions_actors[ii]);
        values[ii] = value_[ii]->read(in, &dummy);
        CRAWL("MultiPolicy::ii: " << ii << " actions_actors: " << actions_actors[ii].v << " values: " << values[ii]);

        send_actions[ii] = actions_actors[ii].v[0];
      }

      for(ii = 0; ii != n_policies; ++ii)
        CRAWL("MultiPolicy::values[ii="<<ii<<"]: " << values[ii]);

      size_t ind = sampler_->sample(values);
      dist = actions_actors[ind].v;

      // CRAWL("Multipolicy::-std::ii_max: " << ii_max << " tmp_action.v: " << tmp_action.v);

    }
    break;

  }

  if (!finite(dist[0]))
  {
    ERROR("Result %f is not finite!" << dist[0]);
    ERROR("n_dimension: " << n_dimension);
  }

  *out = dist;
  out->type = atExploratory;
  
  CRAWL("MultiPolicy::(*out): " << (*out) << "\n");
  
}

