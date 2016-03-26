/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, Carnegie Mellon University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Carnegie Mellon University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/** \author Jeongseok Lee <jslee02@gmail.com> */

#define BOOST_TEST_MODULE "FCL_SHAPE_MESH_CONSISTENCY"
#include <boost/test/unit_test.hpp>

#include "fcl/narrowphase/narrowphase.h"
#include "fcl/shape/geometric_shape_to_BVH_model.h"
#include "fcl/distance.h"
#include "fcl/collision.h"
#include "test_fcl_utility.h"

using namespace fcl;

bool checkBoundingBox(const Vec3f& min, const Vec3f& max,
                      const Vec3f& point, double tol = 1e-12)
{
  for (size_t i = 0u; i < 3u; ++i)
  {
    if (min[i] - tol > point[i] || point[i] > max[i] + tol)
      return false;
  }

  return true;
}

BOOST_AUTO_TEST_CASE(distance_capsule_box)
{
  const Transform3f identity;
  Transform3f pose;

  Box s1(1, 1, 1);
  Box s2(0.5, 0.5, 0.5);

  BVHModel<AABB> s1_aabb;
  BVHModel<AABB> s2_aabb;
  BVHModel<OBB> s1_obb;
  BVHModel<OBB> s2_obb;
  BVHModel<RSS> s1_rss;
  BVHModel<RSS> s2_rss;
  BVHModel<OBBRSS> s1_obbrss;
  BVHModel<OBBRSS> s2_obbrss;

  generateBVHModel(s1_aabb, s1, identity);
  generateBVHModel(s2_aabb, s2, identity);
  generateBVHModel(s1_obb, s1, identity);
  generateBVHModel(s2_obb, s2, identity);
  generateBVHModel(s1_rss, s1, identity);
  generateBVHModel(s2_rss, s2, identity);
  generateBVHModel(s1_obbrss, s1, identity);
  generateBVHModel(s2_obbrss, s2, identity);

  CollisionRequest request(1e+3, true);

  CollisionResult result;

  //----------------------------------------------------------------------------
  // Case1: s2 is completely inside of s1.
  //
  // Primitive collision returns the maximum number of contact points, 4,
  // whereas mesh collision returns no contact.
  //----------------------------------------------------------------------------

//  result.clear();
//  collide(&s1, identity, &s2, identity, request, result);
//  BOOST_CHECK_EQUAL(result.numContacts(), 4);  // maximum contact number of box-box

//  result.clear();
//  collide(&s1_aabb, identity, &s2_aabb, identity, request, result);
//  BOOST_CHECK_EQUAL(result.numContacts(), 0);

//  result.clear();
//  collide(&s1_obb, identity, &s2_obb, identity, request, result);
//  BOOST_CHECK_EQUAL(result.numContacts(), 0);

//  result.clear();
//  collide(&s1_rss, identity, &s2_rss, identity, request, result);
//  BOOST_CHECK_EQUAL(result.numContacts(), 0);

//  result.clear();
//  collide(&s1_obbrss, identity, &s2_obbrss, identity, request, result);
//  BOOST_CHECK_EQUAL(result.numContacts(), 0);

  //----------------------------------------------------------------------------
  // Case2: The left side (facing -y axis) of s2 touching the right side (facing
  // y axis) of s1. The intersection is a face where the size is the same with
  // the face of s2 (0.5 x 0.5).
  //
  // Primitive collision returns the maximum number of contact points, 4,
  // whereas mesh collision returns no contact.
  //----------------------------------------------------------------------------

  pose.setTranslation(Vec3f(0, 0.75, 0));

  Vec3f min(-0.25, 0.5, -0.25);
  Vec3f max(0.25, 0.5, 0.25);

  result.clear();
  collide(&s1, identity, &s2, pose, request, result);
  BOOST_CHECK_EQUAL(result.numContacts(), 4);
  for (size_t i = 0u; i < result.numContacts(); ++i)
  {
    const Contact& contact = result.getContact(i);
    const bool result = checkBoundingBox(min, max, contact.pos);

    BOOST_CHECK(result);
    if (!result)
      std::cout << "contact point: " << contact.pos << "\n";
  }

  result.clear();
  collide(&s1_obb, identity, &s2_obb, pose, request, result);
  BOOST_CHECK_EQUAL(result.numContacts(), 4);
  for (size_t i = 0u; i < result.numContacts(); ++i)
  {
    const Contact& contact = result.getContact(i);
    const bool result = checkBoundingBox(min, max, contact.pos);

    BOOST_CHECK(result);
    if (!result)
      std::cout << "contact point: " << contact.pos << "\n";
  }

//  result.clear();
//  collide(&s1_obb, identity, &s2_obb, pose, request, result);
//  BOOST_CHECK(result.numContacts() > 0);

//  result.clear();
//  collide(&s1_rss, identity, &s2_rss, pose, request, result);
//  BOOST_CHECK(result.numContacts() > 0);

//  result.clear();
//  collide(&s1_obbrss, identity, &s2_obbrss, pose, request, result);
//  BOOST_CHECK(result.numContacts() > 0);
}
