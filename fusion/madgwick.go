package fusion

import (
	"math"

	"github.com/gurupras/quadcopter"
	"github.com/gurupras/quadcopter/constants"
)

type Quaternion struct {
	Q0 float64
	Q1 float64
	Q2 float64
	Q3 float64
}

func (q *Quaternion) Roll() float64 {
	psi := math.Atan2(2*(q.Q0*q.Q1+q.Q2*q.Q3), 1-2*(q.Q1*q.Q1+q.Q2*q.Q2))
	return psi * constants.RadiansToDegrees
}

func (q *Quaternion) Pitch() float64 {
	theta := 2 * (q.Q0*q.Q2 - q.Q3*q.Q1)
	multiplier := 1.0
	if theta < 0 {
		multiplier = -1
	}
	if math.Abs(theta) >= 1 {
		theta = multiplier * (math.Pi / 2)
	} else {
		theta = math.Asin(theta)
	}
	return theta * constants.RadiansToDegrees
}

func (q *Quaternion) Yaw() float64 {
	phi := math.Atan2(2*(q.Q0*q.Q3+q.Q1*q.Q2), 1-2*(q.Q2*q.Q2+q.Q3*q.Q3))
	return phi * constants.RadiansToDegrees
}

func (m *MadgwickAHRS) GetOrientation() (quadcopter.Roll, quadcopter.Pitch, quadcopter.Yaw) {
	roll := quadcopter.Roll(m.Roll())
	pitch := quadcopter.Pitch(m.Pitch())
	yaw := quadcopter.Yaw(m.Yaw())

	return roll, pitch, yaw
}

type MadgwickAHRS struct {
	sampleFreq float64
	Beta       float64
	*Quaternion
}

func NewMadgwickAHRS(freq float64) *MadgwickAHRS {
	return &MadgwickAHRS{
		sampleFreq: freq,
		Beta:       0.1,
		Quaternion: &Quaternion{
			Q0: 1,
			Q1: 0,
			Q2: 0,
			Q3: 0,
		},
	}
}

func (m *MadgwickAHRS) Update(ax, ay, az, gx, gy, gz, mx, my, mz float64) {
	var (
		recipNorm float64
		s0        float64
		s1        float64
		s2        float64
		s3        float64
		qDot1     float64
		qDot2     float64
		qDot3     float64
		qDot4     float64
		hx        float64
		hy        float64
		_2q0mx    float64
		_2q0my    float64
		_2q0mz    float64
		_2q1mx    float64
		_2bx      float64
		_2bz      float64
		_4bx      float64
		_4bz      float64
		_2q0      float64
		_2q1      float64
		_2q2      float64
		_2q3      float64
		_2q0q2    float64
		_2q2q3    float64
		q0q0      float64
		q0q1      float64
		q0q2      float64
		q0q3      float64
		q1q1      float64
		q1q2      float64
		q1q3      float64
		q2q2      float64
		q2q3      float64
		q3q3      float64
	)

	invSampleFreq := 1 / m.sampleFreq

	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	if (mx == 0) && (my == 0) && (mz == 0) {
		m.UpdateIMU(ax, ay, az, gx, gy, gz)
		return
	}

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5 * (-m.Q1*gx - m.Q2*gy - m.Q3*gz)
	qDot2 = 0.5 * (m.Q0*gx + m.Q2*gz - m.Q3*gy)
	qDot3 = 0.5 * (m.Q0*gy - m.Q1*gz + m.Q3*gx)
	qDot4 = 0.5 * (m.Q0*gz + m.Q1*gy - m.Q2*gx)

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if !((ax == 0) && (ay == 0) && (az == 0)) {

		// Normalise accelerometer measurement
		recipNorm = FastInvSqrt64(ax*ax + ay*ay + az*az)
		ax *= recipNorm
		ay *= recipNorm
		az *= recipNorm

		// Normalise magnetometer measurement
		recipNorm = FastInvSqrt64(mx*mx + my*my + mz*mz)
		mx *= recipNorm
		my *= recipNorm
		mz *= recipNorm

		// Auxiliary variables to avoid repeated arithmetic
		_2q0mx = 2.0 * m.Q0 * mx
		_2q0my = 2.0 * m.Q0 * my
		_2q0mz = 2.0 * m.Q0 * mz
		_2q1mx = 2.0 * m.Q1 * mx
		_2q0 = 2.0 * m.Q0
		_2q1 = 2.0 * m.Q1
		_2q2 = 2.0 * m.Q2
		_2q3 = 2.0 * m.Q3
		_2q0q2 = 2.0 * m.Q0 * m.Q2
		_2q2q3 = 2.0 * m.Q2 * m.Q3
		q0q0 = m.Q0 * m.Q0
		q0q1 = m.Q0 * m.Q1
		q0q2 = m.Q0 * m.Q2
		q0q3 = m.Q0 * m.Q3
		q1q1 = m.Q1 * m.Q1
		q1q2 = m.Q1 * m.Q2
		q1q3 = m.Q1 * m.Q3
		q2q2 = m.Q2 * m.Q2
		q2q3 = m.Q2 * m.Q3
		q3q3 = m.Q3 * m.Q3

		// Reference direction of Earth's magnetic field
		hx = mx*q0q0 - _2q0my*m.Q3 + _2q0mz*m.Q2 + mx*q1q1 + _2q1*my*m.Q2 + _2q1*mz*m.Q3 - mx*q2q2 - mx*q3q3
		hy = _2q0mx*m.Q3 + my*q0q0 - _2q0mz*m.Q1 + _2q1mx*m.Q2 - my*q1q1 + my*q2q2 + _2q2*mz*m.Q3 - my*q3q3
		_2bx = math.Sqrt(hx*hx + hy*hy)
		_2bz = -_2q0mx*m.Q2 + _2q0my*m.Q1 + mz*q0q0 + _2q1mx*m.Q3 - mz*q1q1 + _2q2*my*m.Q3 - mz*q2q2 + mz*q3q3
		_4bx = 2.0 * _2bx
		_4bz = 2.0 * _2bz

		// Gradient decent algorithm corrective step
		s0 = -_2q2*(2.0*q1q3-_2q0q2-ax) + _2q1*(2.0*q0q1+_2q2q3-ay) - _2bz*m.Q2*(_2bx*(0.5-q2q2-q3q3)+_2bz*(q1q3-q0q2)-mx) + (-_2bx*m.Q3+_2bz*m.Q1)*(_2bx*(q1q2-q0q3)+_2bz*(q0q1+q2q3)-my) + _2bx*m.Q2*(_2bx*(q0q2+q1q3)+_2bz*(0.5-q1q1-q2q2)-mz)
		s1 = _2q3*(2.0*q1q3-_2q0q2-ax) + _2q0*(2.0*q0q1+_2q2q3-ay) - 4.0*m.Q1*(1-2.0*q1q1-2.0*q2q2-az) + _2bz*m.Q3*(_2bx*(0.5-q2q2-q3q3)+_2bz*(q1q3-q0q2)-mx) + (_2bx*m.Q2+_2bz*m.Q0)*(_2bx*(q1q2-q0q3)+_2bz*(q0q1+q2q3)-my) + (_2bx*m.Q3-_4bz*m.Q1)*(_2bx*(q0q2+q1q3)+_2bz*(0.5-q1q1-q2q2)-mz)
		s2 = -_2q0*(2.0*q1q3-_2q0q2-ax) + _2q3*(2.0*q0q1+_2q2q3-ay) - 4.0*m.Q2*(1-2.0*q1q1-2.0*q2q2-az) + (-_4bx*m.Q2-_2bz*m.Q0)*(_2bx*(0.5-q2q2-q3q3)+_2bz*(q1q3-q0q2)-mx) + (_2bx*m.Q1+_2bz*m.Q3)*(_2bx*(q1q2-q0q3)+_2bz*(q0q1+q2q3)-my) + (_2bx*m.Q0-_4bz*m.Q2)*(_2bx*(q0q2+q1q3)+_2bz*(0.5-q1q1-q2q2)-mz)
		s3 = _2q1*(2.0*q1q3-_2q0q2-ax) + _2q2*(2.0*q0q1+_2q2q3-ay) + (-_4bx*m.Q3+_2bz*m.Q1)*(_2bx*(0.5-q2q2-q3q3)+_2bz*(q1q3-q0q2)-mx) + (-_2bx*m.Q0+_2bz*m.Q2)*(_2bx*(q1q2-q0q3)+_2bz*(q0q1+q2q3)-my) + _2bx*m.Q1*(_2bx*(q0q2+q1q3)+_2bz*(0.5-q1q1-q2q2)-mz)
		recipNorm = FastInvSqrt64(s0*s0 + s1*s1 + s2*s2 + s3*s3) // normalise step magnitude
		s0 *= recipNorm
		s1 *= recipNorm
		s2 *= recipNorm
		s3 *= recipNorm

		// Apply feedback step
		qDot1 -= m.Beta * s0
		qDot2 -= m.Beta * s1
		qDot3 -= m.Beta * s2
		qDot4 -= m.Beta * s3
	}

	// Integrate rate of change of quaternion to yield quaternion
	m.Q0 += qDot1 * invSampleFreq
	m.Q1 += qDot2 * invSampleFreq
	m.Q2 += qDot3 * invSampleFreq
	m.Q3 += qDot4 * invSampleFreq

	// Normalise quaternion
	recipNorm = FastInvSqrt64(m.Q0*m.Q0 + m.Q1*m.Q1 + m.Q2*m.Q2 + m.Q3*m.Q3)
	m.Q0 *= recipNorm
	m.Q1 *= recipNorm
	m.Q2 *= recipNorm
	m.Q3 *= recipNorm
}

func (m *MadgwickAHRS) UpdateIMU(ax, ay, az, gx, gy, gz float64) {
	var (
		recipNorm float64
		s0        float64
		s1        float64
		s2        float64
		s3        float64
		qDot1     float64
		qDot2     float64
		qDot3     float64
		qDot4     float64
		_2q0      float64
		_2q1      float64
		_2q2      float64
		_2q3      float64
		_4q0      float64
		_4q1      float64
		_4q2      float64
		_8q1      float64
		_8q2      float64
		q0q0      float64
		q1q1      float64
		q2q2      float64
		q3q3      float64
	)

	invSampleFreq := 1 / m.sampleFreq

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5 * (-m.Q1*gx - m.Q2*gy - m.Q3*gz)
	qDot2 = 0.5 * (m.Q0*gx + m.Q2*gz - m.Q3*gy)
	qDot3 = 0.5 * (m.Q0*gy - m.Q1*gz + m.Q3*gx)
	qDot4 = 0.5 * (m.Q0*gz + m.Q1*gy - m.Q2*gx)

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if !((ax == 0.0) && (ay == 0.0) && (az == 0.0)) {

		// Normalise accelerometer measurement
		recipNorm = FastInvSqrt64(ax*ax + ay*ay + az*az)
		ax *= recipNorm
		ay *= recipNorm
		az *= recipNorm

		// Auxiliary variables to avoid repeated arithmetic
		_2q0 = 2.0 * m.Q0
		_2q1 = 2.0 * m.Q1
		_2q2 = 2.0 * m.Q2
		_2q3 = 2.0 * m.Q3
		_4q0 = 4.0 * m.Q0
		_4q1 = 4.0 * m.Q1
		_4q2 = 4.0 * m.Q2
		_8q1 = 8.0 * m.Q1
		_8q2 = 8.0 * m.Q2
		q0q0 = m.Q0 * m.Q0
		q1q1 = m.Q1 * m.Q1
		q2q2 = m.Q2 * m.Q2
		q3q3 = m.Q3 * m.Q3

		// Gradient decent algorithm corrective step
		s0 = _4q0*q2q2 + _2q2*ax + _4q0*q1q1 - _2q1*ay
		s1 = _4q1*q3q3 - _2q3*ax + 4.0*q0q0*m.Q1 - _2q0*ay - _4q1 + _8q1*q1q1 + _8q1*q2q2 + _4q1*az
		s2 = 4.0*q0q0*m.Q2 + _2q0*ax + _4q2*q3q3 - _2q3*ay - _4q2 + _8q2*q1q1 + _8q2*q2q2 + _4q2*az
		s3 = 4.0*q1q1*m.Q3 - _2q1*ax + 4.0*q2q2*m.Q3 - _2q2*ay
		recipNorm = FastInvSqrt64(s0*s0 + s1*s1 + s2*s2 + s3*s3) // normalise step magnitude
		s0 *= recipNorm
		s1 *= recipNorm
		s2 *= recipNorm
		s3 *= recipNorm

		// Apply feedback step
		qDot1 -= m.Beta * s0
		qDot2 -= m.Beta * s1
		qDot3 -= m.Beta * s2
		qDot4 -= m.Beta * s3
	}

	// Integrate rate of change of quaternion to yield quaternion
	m.Q0 += qDot1 * invSampleFreq
	m.Q1 += qDot2 * invSampleFreq
	m.Q2 += qDot3 * invSampleFreq
	m.Q3 += qDot4 * invSampleFreq

	// Normalise quaternion
	recipNorm = FastInvSqrt64(m.Q0*m.Q0 + m.Q1*m.Q1 + m.Q2*m.Q2 + m.Q3*m.Q3)
	m.Q0 *= recipNorm
	m.Q1 *= recipNorm
	m.Q2 *= recipNorm
	m.Q3 *= recipNorm
}

// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
// https://github.com/arccoza/go-fastinvsqrt

const magic64 = 0x5FE6EB50C7B537A9

func FastInvSqrt64(n float64) float64 {
	// if n < 0 {
	// 	return math.NaN()
	// }
	// n2, th := n*0.5, float64(1.5)
	// b := math.Float64bits(n)
	// b = magic64 - (b >> 1)
	// f := math.Float64frombits(b)
	// f *= th - (n2 * f * f)
	// return f
	return 1.0 / math.Sqrt(n)
}
