#ifndef LOAM_MATH_UTILS_H
#define LOAM_MATH_UTILS_H


#include "loam_velodyne/Angle.h"
#include "loam_velodyne/Vector3.h"

#include <cmath>


namespace loam {

/** \brief Convert the given radian angle to degrees.
 *
 * @param radians The radian angle to convert.
 * @return The angle in degrees.
 */
inline double rad2deg(double radians)
{
  return radians * 180.0 / M_PI;
}



/** \brief Convert the given radian angle to degrees.
 *
 * @param radians The radian angle to convert.
 * @return The angle in degrees.
 */
inline float rad2deg(float radians)
{
  return (float) (radians * 180.0 / M_PI);
}



/** \brief Convert the given degree angle to radian.
 *
 * @param degrees The degree angle to convert.
 * @return The radian angle.
 */
inline double deg2rad(double degrees)
{
  return degrees * M_PI / 180.0;
}



/** \brief Convert the given degree angle to radian.
 *
 * @param degrees The degree angle to convert.
 * @return The radian angle.
 */
inline float deg2rad(float degrees)
{
  return (float) (degrees * M_PI / 180.0);
}




/** \brief Calculate the squared difference of the given two points.
 *
 * @param a The first point.
 * @param b The second point.
 * @return The squared difference between point a and b.
 */
template <typename PointT>
inline float calcSquaredDiff(const PointT& a, const PointT& b)
{
  float diffX = a.x - b.x;
  float diffY = a.y - b.y;
  float diffZ = a.z - b.z;

  return diffX * diffX + diffY * diffY + diffZ * diffZ;
}



/** \brief Calculate the squared difference of the given two points.
 *
 * @param a The first point.
 * @param b The second point.
 * @param wb The weighting factor for the SECOND point.
 * @return The squared difference between point a and b.
 */
template <typename PointT>
inline float calcSquaredDiff(const PointT& a, const PointT& b, const float& wb)
{
  float diffX = a.x - b.x * wb;
  float diffY = a.y - b.y * wb;
  float diffZ = a.z - b.z * wb;

  return diffX * diffX + diffY * diffY + diffZ * diffZ;
}


/** \brief Calculate the absolute distance of the point to the origin.
 *
 * @param p The point.
 * @return The distance to the point.
 */
template <typename PointT>
inline float calcPointDistance(const PointT& p)
{
  return std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
}



/** \brief Calculate the squared distance of the point to the origin.
 *
 * @param p The point.
 * @return The squared distance to the point.
 */
template <typename PointT>
inline float calcSquaredPointDistance(const PointT& p)
{
  return p.x * p.x + p.y * p.y + p.z * p.z;
}



/** \brief Rotate the given vector by the specified angle around the x-axis.
 *
 * @param v the vector to rotate
 * @param ang the rotation angle
 */
inline void rotX(Vector3& v, const Angle& ang)
{
  float y = v.y();
  v.y() = ang.cos() * y - ang.sin() * v.z();
  v.z() = ang.sin() * y + ang.cos() * v.z();
}

/** \brief Rotate the given point by the specified angle around the x-axis.
 *
 * @param p the point to rotate
 * @param ang the rotation angle
 */
template <typename PointT>
inline void rotX(PointT& p, const Angle& ang)
{
  float y = p.y;
  p.y = ang.cos() * y - ang.sin() * p.z;
  p.z = ang.sin() * y + ang.cos() * p.z;
}



/** \brief Rotate the given vector by the specified angle around the y-axis.
 *
 * @param v the vector to rotate
 * @param ang the rotation angle
 */
inline void rotY(Vector3& v, const Angle& ang)
{
  float x = v.x();
  v.x() = ang.cos() * x + ang.sin() * v.z();
  v.z() = ang.cos() * v.z() - ang.sin() * x;
}

/** \brief Rotate the given point by the specified angle around the y-axis.
 *
 * @param p the point to rotate
 * @param ang the rotation angle
 */
template <typename PointT>
inline void rotY(PointT& p, const Angle& ang)
{
  float x = p.x;
  p.x = ang.cos() * x + ang.sin() * p.z;
  p.z = ang.cos() * p.z - ang.sin() * x;
}



/** \brief Rotate the given vector by the specified angle around the z-axis.
 *
 * @param v the vector to rotate
 * @param ang the rotation angle
 */
inline void rotZ(Vector3& v, const Angle& ang)
{
  float x = v.x();
  v.x() = ang.cos() * x - ang.sin() * v.y();
  v.y() = ang.sin() * x + ang.cos() * v.y();
}

/** \brief Rotate the given point by the specified angle around the z-axis.
 *
 * @param p the point to rotate
 * @param ang the rotation angle
 */
template <typename PointT>
inline void rotZ(PointT& p, const Angle& ang)
{
  float x = p.x;
  p.x = ang.cos() * x - ang.sin() * p.y;
  p.y = ang.sin() * x + ang.cos() * p.y;
}



/** \brief Rotate the given vector by the specified angles around the z-, x- respectively y-axis.
 *
 * @param v the vector to rotate
 * @param angZ the rotation angle around the z-axis
 * @param angX the rotation angle around the x-axis
 * @param angY the rotation angle around the y-axis
 */
inline void rotateZXY(Vector3& v,
                      const Angle& angZ,
                      const Angle& angX,
                      const Angle& angY)
{
  rotZ(v, angZ);
  rotX(v, angX);
  rotY(v, angY);
}

/** \brief Rotate the given point by the specified angles around the z-, x- respectively y-axis.
 *
 * @param p the point to rotate
 * @param angZ the rotation angle around the z-axis
 * @param angX the rotation angle around the x-axis
 * @param angY the rotation angle around the y-axis
 */
template <typename PointT>
inline void rotateZXY(PointT& p,
                      const Angle& angZ,
                      const Angle& angX,
                      const Angle& angY)
{
  rotZ(p, angZ);
  rotX(p, angX);
  rotY(p, angY);
}



/** \brief Rotate the given vector by the specified angles around the y-, x- respectively z-axis.
 *
 * @param v the vector to rotate
 * @param angY the rotation angle around the y-axis
 * @param angX the rotation angle around the x-axis
 * @param angZ the rotation angle around the z-axis
 */
inline void rotateYXZ(Vector3& v,
                      const Angle& angY,
                      const Angle& angX,
                      const Angle& angZ)
{
  rotY(v, angY);
  rotX(v, angX);
  rotZ(v, angZ);
}

/** \brief Rotate the given point by the specified angles around the y-, x- respectively z-axis.
 *
 * @param p the point to rotate
 * @param angY the rotation angle around the y-axis
 * @param angX the rotation angle around the x-axis
 * @param angZ the rotation angle around the z-axis
 */
template <typename PointT>
inline void rotateYXZ(PointT& p,
                      const Angle& angY,
                      const Angle& angX,
                      const Angle& angZ)
{
  rotY(p, angY);
  rotX(p, angX);
  rotZ(p, angZ);
}

 /**
    * Line is given by points AB.
    * The result is the distance and the direction to closest point from the third point X.
    */
    inline float getLinePointDistance(const Eigen::Vector3f &A, const Eigen::Vector3f &B,
      const Eigen::Vector3f &X, Eigen::Vector3f &unit_direction) {
      Eigen::Vector3f BXcrossAX = (X-B).cross(X-A);
      float BXcrossAXnorm = BXcrossAX.norm();
      float lengthAB = (A-B).norm();
      unit_direction = -BXcrossAX.cross(B-A) / (BXcrossAXnorm * lengthAB);
      return BXcrossAXnorm / lengthAB;
      }

      inline float getSurfacePointDistance(const Eigen::Vector3f &A, const Eigen::Vector3f &B, const Eigen::Vector3f &C,
          const Eigen::Vector3f &X, Eigen::Vector3f &surfNormal) {
      surfNormal = (B-A).cross(C-A);
      surfNormal.normalize();

      float normalDotA = -surfNormal.dot(A);
      float distance = surfNormal.dot(X) + normalDotA;
      return distance;
  }

  template <typename PointT>
  inline bool getCornerFeatureCoefficients(const PointT &A, const PointT &B,
      const PointT &X, int iterration, PointT &coeff) {
      Eigen::Vector3f direction;
      float distance = getLinePointDistance(A.getVector3fMap(), B.getVector3fMap(), X.getVector3fMap(), direction);

      float weight = 1.0;// 阻尼因子
      if (iterration >= 5) {
      weight = 1 - 1.8f * fabs(distance);// 点到直线距离越小阻尼因子越大
      }

      coeff.getVector3fMap() = direction * weight;
      coeff.intensity = distance * weight;
      return (weight > 0.1 && distance != 0); // 满足阈值(ld2 < 0.5)，将特征点插入
  }

  template <typename PointT>
  inline bool getCornerFeatureCoefficients(const Eigen::Vector3f &A, const Eigen::Vector3f &B, const Eigen::Vector3f &X, PointT &coeff) {
      Eigen::Vector3f direction;
      float distance = getLinePointDistance(A, B, X, direction);

      float weight = 1 - 0.9f * fabs(distance);
      coeff.getVector3fMap() = direction * weight;
      coeff.intensity = distance * weight;
     
      return (weight > 0.1);
  }

  template <typename PointT>
  inline bool getSurfaceFeatureCoefficients(const PointT &A, const PointT &B, const PointT &C,
      const PointT &X, int iterration, PointT &coefficients) {

      Eigen::Vector3f surfNormal;
      float distance = getSurfacePointDistance(A.getVector3fMap(), B.getVector3fMap(), C.getVector3fMap(),
          X.getVector3fMap(), surfNormal);

      float weight = 1;
      if (iterration >= 5) {
      weight = 1 - 1.8 * fabs(distance) / sqrt(X.getVector3fMap().norm());
      }
      coefficients.getVector3fMap() = weight * surfNormal;
      coefficients.intensity = weight * distance;
      //std::cout<<"weight:"<<weight<<",distance:"<<distance<<std::endl;
      return (weight > 0.1 && distance != 0);
  }

  template <typename PointT>
  inline bool getSurfaceFeatureCoefficients(const Eigen::Vector4f &planeCoef, const PointT &X, PointT &coefficients) {
      float distance = planeCoef.head(3).dot(X.getVector3fMap()) + planeCoef(3);  
      float weight = 1 - 0.9 * fabs(distance) / sqrt(X.getVector3fMap().norm());

      coefficients.getVector3fMap() = planeCoef.head(3) * weight;
      coefficients.intensity = distance * weight;
      return (weight > 0.1);
  }

} // end namespace loam


#endif // LOAM_MATH_UTILS_H
