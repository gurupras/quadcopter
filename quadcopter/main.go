package main

import (
	"bufio"
	"fmt"
	"os"

	"github.com/alecthomas/kingpin"
	"github.com/gurupras/gocommons"
	"github.com/gurupras/quadcopter"
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

	motorStartCmdId    *int
	motorSpeedCmdId    *int
	motorSpeedCmdSpeed *int
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
	motorStartCmdId = motorStartCmd.Arg("id", "ID of motor").Required().Int()

	motorSpeedCmd = motorCmd.Command("speed", "Set speed of motor")
	motorSpeedCmdId = motorSpeedCmd.Arg("id", "ID of motor").Required().Int()
	motorSpeedCmdSpeed = motorSpeedCmd.Arg("speed", "Speed of motor").Required().Int()

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
			for _, esc := range quad.Esc {
				esc.Init()
				go esc.AsyncStart()
			}
		case motorStartCmd.FullCommand():
			if quad == nil {
				fmt.Fprintln(os.Stderr, "Issue start cmd first")
				continue
			}
			if esc := quad.GetEsc(*motorStartCmdId); esc == nil {
				fmt.Fprintln(os.Stderr, "Invalid motor ID. Use 1, 2, 3, 4")
				continue
			} else {
				esc.Init()
			}

		case motorSpeedCmd.FullCommand():
			if quad == nil {
				fmt.Fprintln(os.Stderr, "Issue start cmd first")
				continue
			}
			if esc := quad.GetEsc(*motorStartCmdId); esc == nil {
				fmt.Fprintln(os.Stderr, "Invalid motor ID. Use 1, 2, 3, 4")
				continue
			} else {
				esc.SetSpeed(*motorSpeedCmdSpeed)
			}
		}
	}
}
