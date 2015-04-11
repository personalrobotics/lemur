/* File: test2dimg.cpp
 * Author: Chris Dellin <cdellin@gmail.com>
 * Copyright: 2015 Carnegie Mellon University
 * License: BSD
 */

#include <cstdio>
#include <cstdlib>
#include <png.h>
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <ompl/base/goals/GoalStates.h>
#include <ompl/base/Planner.h>
#include <ompl/base/PlannerTerminationCondition.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

#include <ompl_multiset/Roadmap.h>
#include <ompl_multiset/RoadmapSampledConst.h>
#include <ompl_multiset/Cache.h>
#include <ompl_multiset/MultiSetPRM.h>

int read_png(const char * filename, int * pwidth, int * pheight, unsigned char ** pdata)
{
   bool success;
   int i;
   FILE * fp = 0;
   png_structp png_ptr = 0;
   png_infop info_ptr = 0;
   int ret = 0;
   png_uint_32 width;
   png_uint_32 height;
   int bit_depth;
   int color_type;
   unsigned char * data = 0;
   unsigned char ** row_pointers;
   
   fp = fopen(filename, "rb");
   if (!fp)
   {
      fprintf(stderr,"file open failed!\n");
      ret = -1;
      goto error;
   }
   
   //png_ptr = png_create_read_struct(PNG_LIBPNG_VER_STRING, 0, &user_error_fn, 0);
   png_ptr = png_create_read_struct(PNG_LIBPNG_VER_STRING, 0, 0, 0);
   if (!png_ptr)
   {
      fprintf(stderr,"png_create_read_struct failed!\n");
      ret = -2;
      goto error;
   }
   
   info_ptr = png_create_info_struct(png_ptr);
   if (!info_ptr)
   {
      fprintf(stderr,"png_create_info_struct failed!\n");
      ret = -3;
      goto error;
   }
   
   if (setjmp(png_jmpbuf(png_ptr)))
   {
      fprintf(stderr,"some type of png error, jumped!\n");
      ret = -4;
      goto error;
   }
   
   png_init_io(png_ptr, fp);
   png_read_png(png_ptr, info_ptr, PNG_TRANSFORM_IDENTITY, 0);
   
   success = (1==png_get_IHDR(png_ptr, info_ptr, &width, &height,
      &bit_depth, &color_type, 0, 0, 0));
   if (!success)
   {
      fprintf(stderr,"png_get_IHDR failed!\n");
      ret = -5;
      goto error;
   }
   if (bit_depth != 8 || color_type != PNG_COLOR_TYPE_RGB)
   {
      fprintf(stderr,"image type is not 8-bit RGB!\n");
      ret = -6;
      goto error;
   }
   
   if (3 != png_get_channels(png_ptr, info_ptr))
   {
      fprintf(stderr,"expected 3 channels (RGB)!\n");
      ret = -7;
      goto error;
   }
   
   if (width*3 != png_get_rowbytes(png_ptr, info_ptr))
   {
      fprintf(stderr,"expected %lu bytes per row!\n", width*3);
      ret = -8;
      goto error;
   }
   
   data = (unsigned char *) malloc(width*height*3);
   if (!data)
   {
      fprintf(stderr,"memory alloc error!\n");
      ret = -9;
      goto error;
   }
   
   row_pointers = png_get_rows(png_ptr, info_ptr);
   for (i=0; i<(int)height; i++)
   {
      memcpy(data+i*width*3, row_pointers[i], width*3);
   }
   
   *pwidth = width;
   *pheight = height;
   *pdata = data;
   data = 0;
   
error:
   free(data);
   if (info_ptr) png_destroy_read_struct(0, &info_ptr, 0);
   if (png_ptr) png_destroy_read_struct(&png_ptr, 0, 0);
   if (fp) fclose(fp);
   return ret;
}

bool isvalid(unsigned char * data, int width, int height, double * costp, const char * type, const ompl::base::State * s)
{
   int i;
   int j;
   unsigned char * px;
   bool isvalid;
   double * q = s->as<ompl::base::RealVectorStateSpace::StateType>()->values;
   i = floor(q[0]);
   j = floor(q[1]);
   px = data + i*width*3 + j*3;
   if (strcmp(type,"r")==0)
   {
      *costp += 10.0;
      if (px[0]==0 && px[1]==0 && px[2]==0)
         isvalid = false;
      else
         isvalid = true;
   }
   else if (strcmp(type,"p")==0)
   {
      *costp += 1.0;
      if (px[0]==0 && px[1]==0 && px[2]==0)
         isvalid = false;
      else if (px[0]==128 && px[1]==128 && px[2]==128)
         isvalid = false;
      else
         isvalid = true;
   }
   else if (strcmp(type,"rbn")==0)
   {
      if (px[0]==0 && px[1]==0 && px[2]==0) /* black */
      {
         *costp += 11.0;
         isvalid = false;
      }
      else if (px[0]==128 && px[1]==128 && px[2]==128) /* grey */
      {
         *costp += 11.0;
         isvalid = true;
      }
      else /* assumed white */
      {
         *costp += 1.0;
         isvalid = true;
      }
   }
   else
   {
      fprintf(stderr, "type %s not known!\n", type);
      exit(-1);
   }
   //printf("    checking %d,%d in %s ... %s!\n", i, j, type, isvalid ? "yes" : "no");
   return isvalid;
}

int main(int argc, char * argv[])
{
   int width;
   int height;
   int i;
   int j;
   unsigned char * data;
   double cost;
   
   printf("starting test2dimg ...\n");
   
   if (argc != 6)
   {
      fprintf(stderr,"Usage: %s <image> <type> <seed> <dumpfile> <lambda>!\n", argv[0]);
      fprintf(stderr,"type is [r|rp-naive|rp-checkmask]\n");
      exit(1);
   }
   
   //ompl::RNG::setSeed(atoi(argv[3]));
   
   read_png(argv[1], &width, &height, &data);
   printf("dims: %dx%d\n", width, height);
   
   /* state space (2D circle+cross) */
   ompl::base::StateSpacePtr space(new ompl::base::RealVectorStateSpace(2));
   /* state space set bounds */
   {
      ompl::base::RealVectorBounds bounds(2);
      bounds.setLow(0, 0.0); bounds.setHigh(0, height);
      bounds.setLow(1, 0.0); bounds.setHigh(1, width);
      space->as<ompl::base::RealVectorStateSpace>()->setBounds(bounds);
   }
   /* set space resolution */
   space->setLongestValidSegmentFraction(2.0 / space->getMaximumExtent());
   
   /* create planner */
   ompl_multiset::RoadmapPtr roadmap(
      new ompl_multiset::RoadmapSampledConst(space, atoi(argv[3]), 100, 100.0));
   ompl_multiset::MultiSetPRM * p
      = ompl_multiset::MultiSetPRM::create(space, roadmap);
   p->set_interroot_radius(100.0);
   p->set_lambda(atof(argv[5]));
   p->set_dumpfile(argv[4]);
   
   /* create spaceinfos */
   ompl::base::SpaceInformationPtr si_r(new ompl::base::SpaceInformation(space));
   si_r->setStateValidityChecker(boost::bind(isvalid, data, width, height, &cost, "r", _1));
   p->add_cfree(si_r, "r", 10.0);
   ompl::base::SpaceInformationPtr si_p(new ompl::base::SpaceInformation(space));
   si_p->setStateValidityChecker(boost::bind(isvalid, data, width, height, &cost, "p", _1));
   p->add_cfree(si_p, "p", 1.0);
   
   ompl::base::SpaceInformationPtr si_rbn(new ompl::base::SpaceInformation(space));
   si_rbn->setStateValidityChecker(boost::bind(isvalid, data, width, height, &cost, "rbn", _1));
   p->add_cfree(si_rbn, "rbn", 1.0);
   
   /* create a problem definition from this si_ra container
    * (with start and goal configs) */
   ompl::base::ProblemDefinitionPtr pdef;
   if (strcmp(argv[2],"r")==0 || strcmp(argv[2],"rp-checkmask")==0)
      pdef.reset(new ompl::base::ProblemDefinition(si_r));
   else if (strcmp(argv[2],"rp-naive")==0)
      pdef.reset(new ompl::base::ProblemDefinition(si_rbn));
   else
   {
      fprintf(stderr,"unknown type!\n");
      exit(1);
   }
   
   /* set start/goal states */
   {
      pdef->clearStartStates();
      ompl::base::GoalStates * gs = new ompl::base::GoalStates(si_r);
      gs->clear();
      for (i=0; i<height; i++)
      for (j=0; j<width; j++)
      {
         unsigned char * px = data + i*width*3 + j*3;
         if ((px[0]==0) && (px[1]==255) & (px[2]==0))
         {
            ompl::base::ScopedState<ompl::base::RealVectorStateSpace> s(space);
            s->values[0] = i + 0.5;
            s->values[1] = j + 0.5;
            printf("found a start at %f,%f!\n", s->values[0], s->values[1]);
            pdef->addStartState(s);
         }
         if ((px[0]==0) && (px[1]==0) & (px[2]==255))
         {
            ompl::base::ScopedState<ompl::base::RealVectorStateSpace> s(space);
            s->values[0] = i + 0.5;
            s->values[1] = j + 0.5;
            printf("found a goal at %f,%f!\n", s->values[0], s->values[1]);
            gs->addState(s);
         }
      }
      pdef->clearGoal();
      pdef->setGoal(ompl::base::GoalPtr(gs));
   }
   
   if (strcmp(argv[2],"rp-checkmask")==0)
      p->add_inclusion(si_r, si_p);
   
   p->update_subsets();
   
   cost = 0.0;
   p->setProblemDefinition(pdef);
   ompl::base::PlannerStatus status = p->solve(ompl::base::timedPlannerTerminationCondition(600.0));
   if (status != ompl::base::PlannerStatus::EXACT_SOLUTION)
   {
      fprintf(stderr, "no solution found after 600 seconds!\n");
      exit(1);
   }
   
   delete p;
   
   {
      FILE * fp;
      fp = fopen(argv[4], "a");
      fprintf(fp, "final_check_cost %f\n", cost);
      fclose(fp);
   }
   
   printf("solved! total cost: %f\n", cost);
   
   return 0;
}
