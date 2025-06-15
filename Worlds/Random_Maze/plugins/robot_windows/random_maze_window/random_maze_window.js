import RobotWindow from 'https://cyberbotics.com/wwi/R2025a/RobotWindow.js';

// Function to log messages to the HTML console
window.log = function(message) {
  const consoleUl = document.getElementById('console');
  const li = document.createElement('li');
  li.appendChild(document.createTextNode(message));
  consoleUl.appendChild(li);
  consoleUl.scrollTop = consoleUl.scrollHeight;
}

// Function to send a "randomize_maze" command
window.randomizeMaze = function() {
  const message = "randomize_maze";
  window.robotWindow.send(message);
  log(`Sent: ${message}`);
}

function receive(message, robot) {
  log(`Received from robot: ${message}`);
}

window.onload = function() {
  log('HTML page loaded. Initializing RobotWindow...');
  window.robotWindow = new RobotWindow();
  window.robotWindow.setTitle('Obstacle UI');
  window.robotWindow.receive = receive;
  log('RobotWindow initialized.');
};
