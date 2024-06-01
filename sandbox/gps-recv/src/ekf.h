#pragma once

#include <Eigen/Dense>

namespace math {

template<int N, int M>
using Matrix = Eigen::Matrix<float, N, M>;
template<int N>
using Vector = Eigen::Vector<float, N>;

template<int X>
class EKF {
	Matrix<X, X> P;
	Vector<X> x;

public:

	class SystemModel {
	public:
		virtual Vector<X> f(const Vector<X> &x) const = 0;
		virtual Matrix<X, X> df(const Vector<X> &x) const = 0;
	};

	template<int Z>
	class MeasurementModel {
		const Matrix<Z, Z> R;

	protected:
		MeasurementModel(const Matrix<Z, Z> &R) : R{R} {}

	public:
		virtual Vector<Z> h(const Vector<X> &x) const = 0;
		virtual Matrix<Z, X> dh(const Vector<X> &x) const = 0;

		const Matrix<Z, Z> & getR() const {
			return R;
		}
	};

	EKF(const Vector<X> &x_init) : P{Matrix<X, X>::Identity()*1e+6}, x{x_init} {

	}

	void predict(const SystemModel &system) {
		const Matrix<X, X> F = system.df(x);

		x = system.f(x);
		P = F*P*F.transpose();
	}

	template<int Z>
	void correct(const MeasurementModel<Z> &measurement, const Vector<Z> z) {
		const Matrix<Z, X> H = measurement.dh(x);
		const Matrix<X, Z> HT = H.transpose();

		const Vector<Z> y = z - measurement.h(x);
		const Matrix<Z, Z> S = H*P*HT + measurement.getR();
		const Matrix<X, Z> K = P*HT*S.inverse();

		x = x + K*y;
		P = (Matrix<X, X>::Identity() - K*H)*P;
	}

	const Vector<X> & getState() const {
		return x;
	}
};

}
