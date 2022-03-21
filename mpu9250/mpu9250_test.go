package mpu9250

import (
	"testing"
	"time"
)

func TestMPU9250Basic(t *testing.T) {
	dev := New(1)

	for i := 0; i < 100; i++ {
		dev.ReadSample()
		time.Sleep(30 * time.Millisecond)
	}
}
