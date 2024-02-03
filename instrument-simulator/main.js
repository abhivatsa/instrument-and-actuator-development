//-------------------- Websocket -----------------------
let socket = new WebSocket("ws://127.0.0.1:8080");
// var btn = document.getElementById("btn");
socket.onopen = function (e) {
    console.log("[open] Connection established");
    console.log("Sending to server");
    mychart.update();
};

socket.onmessage = function (event) {
    // console.log(`[message] Data received from server`);
    let incoming_data = JSON.parse(event.data);
    // console.log(incoming_data)
    updateRobotState(incoming_data.current_state.positions, incoming_data.current_state.velocities, incoming_data.current_state.torque);
    updateSystemState(incoming_data.system_state);
      
};

socket.onclose = function (event) {
    if (event.wasClean) {
        console.log(`[close] Connection closed cleanly, code=${event.code} reason=${event.reason}`);
    } else {
        // e.g. server process killed or network down
        // event.code is usually 1006 in this case
        console.log('[close] Connection died');
    }
};

socket.onerror = function (error) {
    console.log(`[error]`);
};

// -----------------------------------------------------------------------------------------------------

var current_time = 0;
var joint_positions = [0,0,0,0];
var joint_velocities = [0,0,0,0];
var joint_torque = [0,0,0,0];
// Update robot state
var update_cart_data_done = false;


function updateRobotState(positions, velocities, torque)
{
    joint_val_1.innerText = positions.joint1.toFixed(2);
    joint_val_2.innerText = positions.joint2.toFixed(2);
    joint_val_3.innerText = positions.joint3.toFixed(2);
    joint_val_4.innerText = positions.joint4.toFixed(2);

    j1_slider.value = convertValToRange(positions.joint1, 0);
    j2_slider.value = convertValToRange(positions.joint2, 0);
    j3_slider.value = convertValToRange(positions.joint3, 0);
    j4_slider.value = convertValToRange(positions.joint4, 0);


    current_time = current_time + 0.001;
    joint_positions[0] = positions.joint1;
    joint_positions[1] = positions.joint2;
    joint_positions[2] = positions.joint3;
    joint_positions[3] = positions.joint4;

    joint_velocities[0] = velocities.joint1;
    joint_velocities[1] = velocities.joint2;
    joint_velocities[2] = velocities.joint3;
    joint_velocities[3] = velocities.joint4;

    joint_torque[0] = torque.joint1;
    joint_torque[1] = torque.joint2;
    joint_torque[2] = torque.joint3;
    joint_torque[3] = torque.joint4;
}

function updateChartData()
{
    if(chart_x_data.length >= 10)
    {
        chart_x_data.shift();
        chart_position1.shift();
        chart_position2.shift();
        chart_position3.shift();
        chart_position4.shift();

        chart_velocity1.shift();
        chart_velocity2.shift();
        chart_velocity3.shift();
        chart_velocity4.shift();

        chart_torque1.shift();
        chart_torque2.shift();
        chart_torque3.shift();
        chart_torque4.shift();
    }

    chart_x_data.push(current_time.toFixed(2));
    chart_position1.push(joint_positions[0].toFixed(2));
    chart_position2.push(joint_positions[1].toFixed(2));
    chart_position3.push(joint_positions[2].toFixed(2));
    chart_position4.push(joint_positions[3].toFixed(2));

    chart_velocity1.push(joint_velocities[0].toFixed(2));
    chart_velocity2.push(joint_velocities[1].toFixed(2));
    chart_velocity3.push(joint_velocities[2].toFixed(2));
    chart_velocity4.push(joint_velocities[3].toFixed(2));

    chart_torque1.push(joint_torque[0].toFixed(2));
    chart_torque2.push(joint_torque[1].toFixed(2));
    chart_torque3.push(joint_torque[2].toFixed(2));
    chart_torque4.push(joint_torque[3].toFixed(2));

    mychart.data.labels = chart_x_data;
    if(show_chart_data == ShowChartData.Torque)
    {
        mychart.data.datasets[0].data = chart_torque1;
        mychart.data.datasets[1].data = chart_torque2;
        mychart.data.datasets[2].data = chart_torque3;
        mychart.data.datasets[3].data = chart_torque4;

        mychart.options.scales.yAxes[0].ticks.max = 10;
        mychart.options.scales.yAxes[0].ticks.min = -10;
        mychart.options.scales.yAxes[0].scaleLabel.labelString = "Joint torque (Nm)";
    }
    else if(show_chart_data == ShowChartData.Velocity)
    {
        mychart.data.datasets[0].data = chart_velocity1;
        mychart.data.datasets[1].data = chart_velocity2;
        mychart.data.datasets[2].data = chart_velocity3;
        mychart.data.datasets[3].data = chart_velocity4;

        mychart.options.scales.yAxes[0].ticks.max = 4;
        mychart.options.scales.yAxes[0].ticks.min = -4;
        mychart.options.scales.yAxes[0].scaleLabel.labelString = "Joint velocity (rad/s)";
    }
    else
    {
        mychart.data.datasets[0].data = chart_position1;
        mychart.data.datasets[1].data = chart_position2;
        mychart.data.datasets[2].data = chart_position3;
        mychart.data.datasets[3].data = chart_position4;

        mychart.options.scales.yAxes[0].ticks.max = 4;
        mychart.options.scales.yAxes[0].ticks.min = -4;
        mychart.options.scales.yAxes[0].scaleLabel.labelString = "Joint position (rad)";
    }
    mychart.update();
}

// Update system state
var prev_state = 0;
function updateSystemState(state) 
{
    console.log("state.power_on_status", state.power_on_status);
    if(prev_state == state.power_on_status)
    {
        return;
    }
    // update the data    
    if(state.power_on_status == 3) // power on
    {
        enableButtons(true);
        enablePowerBtn(true);
        powerBtn.checked = true;
        // systemStateSpinner.classList.add("visually-hidden");
        system_state_progress.parentElement.classList.add("visually-hidden");
        systemStateText.innerHTML = "Ready";

        console.log("power on");
    }
    else if(state.power_on_status == 0) // power off
    {
        enableButtons(false);
        enablePowerBtn(true);

        // systemStateSpinner.classList.add("visually-hidden");
        system_state_progress.parentElement.classList.add("visually-hidden");
        systemStateText.innerHTML = "Powered OFF";

        console.log("power off");
    }
    else if(state.power_on_status == 1) // initializing system
    {
        enableButtons(false);
        enablePowerBtn(false);

        // systemStateSpinner.classList.remove("visually-hidden");
        systemStateText.innerHTML = "Initializing System...";
        system_state_progress.parentElement.classList.remove("visually-hidden");
        // system_state_progress.getAttribute("aria-valuenow") = 25;

        console.log("Initializing System...");
    }
    else if(state.power_on_status == 2) // harware check
    {
        enableButtons(false);
        enablePowerBtn(false);

        // systemStateSpinner.classList.remove("visually-hidden");
        systemStateText.textContent = "Hardware check...";

        system_state_progress.parentElement.classList.remove("visually-hidden");
        // system_state_progress.getAttribute("aria-valuenow") = 50;

        console.log("Hardware check...");
    }
    else // In execution
    {
        enableButtons(false);
        enablePowerBtn(false);
        systemStateSpinner.classList.remove("visually-hidden");
        systemStateText.textContent = "Executing...";

        console.log("Executing...");
    }
    prev_state = state.power_on_status;
}

// Send json on Power button click
function powerBtnClicked()
{
    console.log("btn clicked");
    if(!powerBtn.checked)
    {
        enableButtons(false);
    }
    var cmd_obj =
    { 
        system_data : {
            power_on: powerBtn.checked,
            }
    };
    socket.send(JSON.stringify(cmd_obj));
    console.log(cmd_obj);
}

// Send json on Jog btn click
function onJogClicked(mode, index, dir)
{
    // send jog commands
    if(!checkState())
    {
        console.log("mode", mode);
        var m = 2;
        if(mode == "joint_space")
        {
             m = 0;
        }
        else if(mode == "task_space"){
            m = 1;
        } 
        var cmd_obj =
        { 
            command_data : {
                type: 1,
                mode: m,
                index: parseInt(index),
                direction: parseInt(dir)
                }
        };
        socket.send(JSON.stringify(cmd_obj));
    }
}

// Click and hold jog button callback
var t;
function onClickAndHold (mode, index, dir) {
    onJogClicked(mode, index, dir);
    t = setInterval(onJogClicked, 1000, mode, index, dir);
}


// Mode of operation change handle
function onModeChangeClicked(element)
{
    if(checkState(element))
    {
        element.checked = false;
    }
    if(checkState())
    {
        enablePowerBtn(false);
        
    }
    else
    {
        enablePowerBtn(true);
    }
}

function convertValToRange(val, type)
{
    if(type == 0) // radians
    {
        return val*800000/(2*Math.PI);
    }
    else
    {
        return val*800000/2.0;
    }
}

//------------------------DOM Manipulation-----------------------------------
// // Define event handler
const handler = e => {
    console.log(`Document is ready!`)
    
    // disable the buttons
    enableButtons(false);

    
}
  
// // Listen for `DOMContentLoaded` event
document.addEventListener('DOMContentLoaded', handler)

const input_control_tab = document.getElementById("inputControl");

// Input sliders
const handControllerMode = document.getElementById("handController");
const powerBtn = document.getElementById("powerOnMode");
const jogButtons = document.getElementsByClassName("jog");


var buttons = [handControllerMode];

// joint values
var joint_val_1 = document.getElementById("joint_val_1");
var joint_val_2 = document.getElementById("joint_val_2");
var joint_val_3 = document.getElementById("joint_val_3");
var joint_val_4 = document.getElementById("joint_val_4");

// cartesian values
var pitch_val = document.getElementById("pitch_val");
var yaw_val = document.getElementById("yaw_val");
var pinch_val = document.getElementById("pinch_val");
var roll_val = document.getElementById("roll_val");

// joint sliders
var j1_slider = document.getElementById("slider_j1");
var j2_slider = document.getElementById("slider_j2");
var j3_slider = document.getElementById("slider_j3");
var j4_slider = document.getElementById("slider_j4");

// Cartesian Sliders
var pitch_slider = document.getElementById("slider_pitch");
var yaw_slider = document.getElementById("slider_yaw");
var pinch_slider = document.getElementById("slider_pinch");
var roll_slider = document.getElementById("slider_roll");

var plot_position = document.getElementById("plot_position");
var plot_velocity = document.getElementById("plot_velocity");
var plot_torque = document.getElementById("plot_torque");

const systemStateText = document.getElementById("systemState");
const systemStateContainer = document.getElementById("systemStateContainer");
const systemStateSpinner = document.getElementById("systemStateSpinner");

// progess bars
const system_state_progress = document.getElementById("SystemStateProgess");

// enable disble the buttons
function enableButtons(state) 
{
    handControllerMode.disabled = !state;

    for (let index = 0; index < jogButtons.length; index++)
    {
        jogButtons[index].disabled = !state;
    }

    if(state)
    {
        input_control_tab.classList.remove("disabled");
    }
    else
    {
        input_control_tab.classList.add("disabled")
    } 
}

// Check state of the buttons
function checkState(element)
{
    if(!powerBtn.checked)
    {
        return !powerBtn.checked;
    }
    return false;
}

// enable/disable power btn
function enablePowerBtn(state) {
    powerBtn.disabled = !state;
}

powerBtn.addEventListener('click', powerBtnClicked);

// jog Button event handler
for (let index = 0; index < jogButtons.length; index++) {
    const element = jogButtons[index];
    // element.onmousedown = ()=> {onClickAndHold(element.getAttribute("mode"),element.getAttribute("index"), element.getAttribute("direction"))};
    element.onmousedown = ()=> {onJogClicked(element.getAttribute("mode"),element.getAttribute("index"), element.getAttribute("direction"))};
    element.onmouseup = function () {
        onJogClicked("none",element.getAttribute("index"), element.getAttribute("direction"));
        clearTimeout(t);
    }
    element.onmouseleave = () => onJogClicked("none",element.getAttribute("index"), element.getAttribute("direction"));
}


// Mode button event handler
handControllerMode .onclick = () => {
    onModeChangeClicked(handControllerMode);
    var cmd_obj =
        { 
            command_data : {
                type: 2,
                }
        };
        socket.send(JSON.stringify(cmd_obj));
};

//-------------------------------------- CHART JS --------------------
var chart_x_data = Array(10).fill(0);

var max_y_limit = 5;
var y_axis_label = "Joint Position (rad)";


var chart_position1 = Array(10).fill(0);
var chart_position2 = Array(10).fill(0);
var chart_position3 = Array(10).fill(0);
var chart_position4 = Array(10).fill(0);

var chart_velocity1 = Array(10).fill(0);
var chart_velocity2 = Array(10).fill(0);
var chart_velocity3 = Array(10).fill(0);
var chart_velocity4 = Array(10).fill(0);

var chart_torque1 = Array(10).fill(0);
var chart_torque2 = Array(10).fill(0);
var chart_torque3 = Array(10).fill(0);
var chart_torque4 = Array(10).fill(0);


const ShowChartData = {
    Position: 'position',
    Velocity: 'velocity',
    Torque: 'torque'
};

var show_chart_data = ShowChartData.Position;

var mychart = new Chart("myChart", {
    type: "line",
    data: {
        labels: Array(10).fill(0),
    datasets: [{
        data: Array(10).fill(0),
        borderColor: "red",
        fill: false,
        label: 'joint 1',
        radius: 1,
        title: "J1",
        },{
        data: Array(10).fill(0),
        borderColor: "green",
        fill: false,
        label: 'joint 2',
        radius: 1
        },{
        data: Array(10).fill(0),
        borderColor: "blue",
        fill: false,
        label: 'joint 3',
        radius: 1
        },{
        data: Array(10).fill(0),
        borderColor: "yellow",
        fill: false,
        label: 'joint 4',
        radius: 1
        }
]
    },
    options: {
        legend: {display: false},
        points:{radius: 0.1, borderWidth: 0.1},
        scales: {
        x: [{
            ticks: {
            callback: function(val) {
                return val.toFixed(2);
            }
            },
        }],
        // xAxes: [
        //     {scaleLabel: {
        //         display: true,
        //         labelString: 'time (s)'
        //       }}
        // ],
        yAxes : [{
            ticks : {
                max : max_y_limit,    
                min : -max_y_limit,
                stepSize: 0.5
            },
            scaleLabel: {
                display: true,
                labelString: y_axis_label
              }
        }]
        },
        animation: false,
        legend: {
                display: true,
                position : "bottom",
                maxSize : {
                    height : 50
                },
            labels : {
                usePointStyle : true
            }
        },
        // events: [] 
    }        
    });


    var updatechart = () =>{updateChartData();};
    var chart_ = setInterval(updatechart, 500);

    plot_position.onclick = ()=>{show_chart_data = ShowChartData.Position;}
    plot_velocity.onclick = ()=>{show_chart_data = ShowChartData.Velocity;}
    plot_torque.onclick = ()=>{show_chart_data = ShowChartData.Torque;}
//--------------------------------------------------------------------------------

