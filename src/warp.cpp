/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob

    Nori is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License Version 3
    as published by the Free Software Foundation.

    Nori is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#include <nori/warp.h>
#include <nori/vector.h>
#include <nori/frame.h>

NORI_NAMESPACE_BEGIN

Point2f Warp::squareToUniformSquare(const Point2f &sample) {
    return sample;
}

float Warp::squareToUniformSquarePdf(const Point2f &sample) {
    return ((sample.array() >= 0).all() && (sample.array() <= 1).all()) ? 1.0f : 0.0f;
}

Point2f Warp::squareToTent(const Point2f &sample) {
    throw NoriException("Warp::squareToTent() is not yet implemented!");
}

float Warp::squareToTentPdf(const Point2f &p) {
    throw NoriException("Warp::squareToTentPdf() is not yet implemented!");
}

Point2f Warp::squareToUniformDisk(const Point2f &sample) {
    float theta = 2 * M_PI * sample.y();
    float r = std::sqrt(sample.x());
    return Point2f(r * std::cos(theta), r * std::sin(theta));
    //throw NoriException("Warp::squareToUniformDisk() is not yet implemented!");
}

float Warp::squareToUniformDiskPdf(const Point2f &p) {
    return (p.x() * p.x() + p.y() * p.y() < 1 ? 1.0f / M_PI : 0.0f);
    //throw NoriException("Warp::squareToUniformDiskPdf() is not yet implemented!");
}

/*
Reference:
http://www.pbr-book.org/3ed-2018/Monte_Carlo_Integration/2D_Sampling_with_Multidimensional_Transformations.html
*/

Vector3f Warp::squareToUniformSphere(const Point2f &sample) {
    //throw NoriException("Warp::squareToUniformSphere() is not yet implemented!");
    float z = 1 - 2 * sample.x();
    float phi = 2 * M_PI * sample.y();
    return Vector3f(
        std::cos(phi) * std::sqrt(1 - z * z),
        std::sin(phi) * std::sqrt(1 - z * z),
        z
    );
}

float Warp::squareToUniformSpherePdf(const Vector3f &v) {
    //throw NoriException("Warp::squareToUniformSpherePdf() is not yet implemented!");
    float r = std::sqrt(v.x() * v.x() + v.y() * v.y() + v.z() * v.z());
    return (r > 1.0f) ? 0.0f : 1.0f / (4 * M_PI);
}

Vector3f Warp::squareToUniformHemisphere(const Point2f &sample) {
    //throw NoriException("Warp::squareToUniformHemisphere() is not yet implemented!");
    float theta = std::acos(1 - sample.x());
    float phi = 2 * M_PI * sample.y();
    return Vector3f(
        std::sin(theta) * std::cos(phi),
        std::sin(theta) * std::sin(phi),
        1 - sample.x()
    );
}

float Warp::squareToUniformHemispherePdf(const Vector3f &v) {
    //throw NoriException("Warp::squareToUniformHemispherePdf() is not yet implemented!");
    float theta = std::acos(v.z());
    float r = std::sqrt(v.x() * v.x() + v.y() * v.y() + v.z() * v.z());
    return (r > 1.0f || theta > M_PI / 2.0f || theta < 0.0f) ? 0.0f : 1.0f / (2 * M_PI);
}

Vector3f Warp::squareToCosineHemisphere(const Point2f &sample) {
    //throw NoriException("Warp::squareToCosineHemisphere() is not yet implemented!");
//    float phi = 2 * M_PI * sample.y();
//    float theta = std::asin(sample.x()) / 2.0f;
//    return Vector3f(
//        std::sin(theta) * std::cos(phi),
//        std::sin(theta) * std::sin(phi),
//        std::cos(theta)
//        );
        float r1 = 2.0f * sample.x() - 1.0f;
		float r2 = 2.0f * sample.y() - 1.0f;
		float phi, r;
		if (r1 == 0 && r2 == 0) {
			r = phi = 0;
		}
		else if (r1 * r1 > r2 * r2) {
			r = r1;
			phi = (M_PI / 4.0f) * (r2 / r1);
		}
		else {
			r = r2;
			phi = (M_PI / 2.0f) - (r1 / r2) * (M_PI / 4.0f);
		}
		float cosPhi = std::cos(phi);
		float sinPhi = std::sin(phi);
		Vector2f t = Vector2f(r * cosPhi, r * sinPhi);
		float z = std::sqrt(std::max(0.0f, 1.0f - t.x()*t.x() - t.y()*t.y()));
		return Vector3f(t.x(), t.y(), z);
}

float Warp::squareToCosineHemispherePdf(const Vector3f &v) {
    //throw NoriException("Warp::squareToCosineHemispherePdf() is not yet implemented!");
    float theta = std::acos(v.z());
    float r = std::sqrt(v.x() * v.x() + v.y() * v.y() + v.z() * v.z());
    return (theta > M_PI / 2.0f || theta < 0.0f || r > 1.0f) ? 0.0f : std::cos(theta) / M_PI;
}

Vector3f Warp::squareToBeckmann(const Point2f &sample, float alpha) {
    throw NoriException("Warp::squareToBeckmann() is not yet implemented!");
}

float Warp::squareToBeckmannPdf(const Vector3f &m, float alpha) {
    throw NoriException("Warp::squareToBeckmannPdf() is not yet implemented!");
}

NORI_NAMESPACE_END
