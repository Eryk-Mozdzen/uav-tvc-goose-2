#pragma once

#include <Eigen/Dense>

template<typename T, int X>
class EKF {
	Eigen::Vector<T, X> x;
	Eigen::Matrix<T, X, X> P;

public:
	template<int U>
	struct SystemModel {
		const Eigen::Matrix<T, X, X> Q;

		SystemModel(const T &Q_value) : Q{Eigen::Matrix<T, X, X>::Identity()*Q_value} {}
		SystemModel(const Eigen::Vector<T, X> &Q_diagonal) : Q{Eigen::DiagonalMatrix<T, X>(Q_diagonal)} {}

		virtual Eigen::Vector<T, X> f(const Eigen::Vector<T, X> &x, const Eigen::Vector<T, U> &u) const = 0;
		virtual Eigen::Matrix<T, X, X> df(const Eigen::Vector<T, X> &x, const Eigen::Vector<T, U> &u) const = 0;
	};

	template<int Z>
	struct MeasurementModel {
		const Eigen::Matrix<T, Z, Z> R;

		MeasurementModel(const T &R_value) : R{Eigen::Matrix<T, Z, Z>::Identity()*R_value} {}
		MeasurementModel(const Eigen::Vector<T, Z> &R_diagonal) : R{Eigen::DiagonalMatrix<T, Z>(R_diagonal)} {}

		virtual Eigen::Vector<T, Z> h(const Eigen::Vector<T, X> &x) const = 0;
		virtual Eigen::Matrix<T, Z, X> dh(const Eigen::Vector<T, X> &x) const = 0;
	};

	EKF(const Eigen::Vector<T, X> &initial) : x{initial}, P{Eigen::Matrix<T, X, X>::Identity()*1e+6} {

	}

	template<int U>
	void predict(const SystemModel<U> &system, const Eigen::Vector<T, U> &u) {
		const Eigen::Matrix F = system.df(x, u);

		x = system.f(x, u);
		P = F*P*F.transpose() + system.Q;
	}

	template<int Z>
	void correct(const MeasurementModel<Z> &measurement, const Eigen::Vector<T, Z> &z) {
		const Eigen::Matrix H = measurement.dh(x);

		const Eigen::Vector<T, Z> y = z - measurement.h(x);
		const Eigen::Matrix<T, Z, Z> S = H*P*H.transpose() + measurement.R;
		const Eigen::Matrix<T, X, Z> K = P*H.transpose()*S.inverse();

		x = x + K*y;
		P = (Eigen::Matrix<T, X, X>::Identity() - K*H)*P;
	}

	const Eigen::Vector<T, X> & getState() const {
		return x;
	}
};
