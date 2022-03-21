package mpu9250

import (
	"testing"
	"time"

	log "github.com/sirupsen/logrus"
	"github.com/stretchr/testify/require"
)

func TestMPU9250Basic(t *testing.T) {
	log.SetLevel(log.DebugLevel)

	require := require.New(t)

	dev := New(1)

	err := dev.Init()
	require.Nil(err)

	for i := 0; i < 100; i++ {
		dev.ReadSample()
		time.Sleep(30 * time.Millisecond)
	}
}
