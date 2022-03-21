package main

import (
	"bufio"
	"fmt"
	"os"
	"strconv"
	"strings"

	"github.com/alecthomas/kingpin"
	"github.com/gurupras/gocommons"
	"github.com/gurupras/quadcopter"
	"github.com/gurupras/quadcopter/mpu6050"
)

var (
	app  *kingpin.Application
	port *int
	Port int

	startCmd *kingpin.CmdClause
	exitCmd  *kingpin.CmdClause
	stopCmd  *kingpin.CmdClause

	motorCmd      *kingpin.CmdClause
	motorStartCmd *kingpin.CmdClause
	motorSpeedCmd *kingpin.CmdClause
	motorIncCmd   *kingpin.CmdClause

	motorStartCmdId    *string
	motorSpeedCmdId    *string
	motorSpeedCmdSpeed *int
	motorIncCmdSpeed   *int
)

func setup_parser() *kingpin.Application {
	app = kingpin.New("Go-Quadcopter", "")
	port = app.Flag("i2c", "I2C Device ID").Default("1").Int()
	return app
}

func command_parser() *kingpin.Application {
	app := kingpin.New("Command", "")

	startCmd = app.Command("start", "Start the quadcopter")

	motorCmd = app.Command("motor", "Issue command to motor")

	motorStartCmd = motorCmd.Command("start", "Start a motor")
	motorStartCmdId = motorStartCmd.Arg("id", "ID of motor").Required().String()

	motorSpeedCmd = motorCmd.Command("speed", "Set speed of motor")
	motorSpeedCmdId = motorSpeedCmd.Arg("id", "ID of motor").Required().String()
	motorSpeedCmdSpeed = motorSpeedCmd.Arg("speed", "Speed of motor").Required().Int()

	motorIncCmd = motorCmd.Command("inc", "Increase/Decrease speed of motor")
	motorIncCmdSpeed = motorIncCmd.Arg("speed", "Speed of motor").Required().Int()

	exitCmd = app.Command("exit", "Stop the quadcopter")
	stopCmd = app.Command("stop", "Stop the quadcopter")
	return app
}

func initQuadcopter() *quadcopter.Quadcopter {
	quad := quadcopter.NewQuadcopter()

	for i := 0; i < 4; i++ {
		escI2cDev := quadcopter.NewI2CDevice(uint8(quadcopter.MOTOR_BASE+i), Port)
		esc := quadcopter.NewESC(escI2cDev)
		quad.Esc = append(quad.Esc, esc)
	}
	itgDev := quadcopter.NewI2CDevice(uint8(mpu6050.ITG3200_ADDR), Port)
	itg := mpu6050.NewItg3200(itgDev)

	adxlDev := quadcopter.NewI2CDevice(uint8(mpu6050.ADXL345_ADDRESS), Port)
	adxl345 := mpu6050.NewAdxl345(adxlDev)

	quad.SensorFusion = mpu6050.NewSensorFusion(itg, adxl345)

	fmt.Println("Quadcopter initialized")
	return quad
}

func main() {
	app := setup_parser()
	cmd, err := app.Parse(os.Args[1:])
	_ = cmd
	if err != nil {
		fmt.Fprintln(os.Stderr, "Failed to parse args")
	}
	Port = *port

	reader := bufio.NewReader(os.Stdin)
	cmdParser := command_parser()

	runESC := func(esc *quadcopter.ESC) {
		fmt.Println("Starting ESC:", esc.Addr)
		esc.AsyncStart()
	}

	var quad *quadcopter.Quadcopter
	for {
		fmt.Printf("$>")
		cmdSlice, err := reader.ReadString('\n')
		cmd := gocommons.SliceArgs(cmdSlice)
		command, err := cmdParser.Parse(cmd)
		if err != nil {
			fmt.Fprintln(os.Stderr, "Failed to parse command")
			continue
		}
		switch kingpin.MustParse(command, err) {
		case startCmd.FullCommand():
			quad = initQuadcopter()
		case motorStartCmd.FullCommand():
			if quad == nil {
				fmt.Fprintln(os.Stderr, "Issue start cmd first")
				continue
			}
			if strings.Compare(*motorStartCmdId, "all") == 0 {
				for i := 0; i < 4; i++ {
					esc := quad.GetEsc(i)
					go runESC(esc)
				}
			} else {
				id, _ := strconv.Atoi(*motorStartCmdId)
				if esc := quad.GetEsc(id); esc == nil {
					fmt.Fprintln(os.Stderr, "Invalid motor ID. Use 1, 2, 3, 4")
					continue
				} else {
					go runESC(esc)
				}
			}
		case motorSpeedCmd.FullCommand():
			if quad == nil {
				fmt.Fprintln(os.Stderr, "Issue start cmd first")
				continue
			}
			if strings.Compare(*motorSpeedCmdId, "all") == 0 {
				for i := 0; i < 4; i++ {
					esc := quad.GetEsc(i)
					esc.SetSpeed(*motorSpeedCmdSpeed)
				}
			} else {
				id, _ := strconv.Atoi(*motorSpeedCmdId)
				if esc := quad.GetEsc(id); esc == nil {
					fmt.Fprintln(os.Stderr, "Invalid motor ID. Use 1, 2, 3, 4")
					continue
				} else {
					fmt.Sprintf("Setting motor: %v speed to %v\n", id, *motorSpeedCmdSpeed)
					esc.SetSpeed(*motorSpeedCmdSpeed)
				}
			}
		case motorIncCmd.FullCommand():
			if quad == nil {
				fmt.Fprintln(os.Stderr, "Issue start cmd first")
				continue
			}

			for i := 0; i < 4; i++ {
				esc := quad.GetEsc(i)
				esc.SetSpeed(esc.GetSpeed() + *motorIncCmdSpeed)
			}
		}
	}
}
