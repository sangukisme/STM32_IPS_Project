<?php

$servername = "localhost";
// REPLACE with your Database name
$dbname = "esp_data";
// REPLACE with Database user
$username = "root";
// REPLACE with Database user password
$password = "park1234";

// Create connection
$conn = new mysqli($servername, $username, $password, $dbname);
// Check connection
if ($conn->connect_error) {
    die("Connection failed: " . $conn->connect_error);
} 

$sql = "SELECT id, value1, value2, value3, value4, reading_time FROM sensor2 order by reading_time desc limit 1";

$result = $conn->query($sql);

while ($data = $result->fetch_assoc()){
    //$sensor_data[] = $data;
    $x1 = $data['value1'];
    $y1 = $data['value2'];
    $x2 = $data['value3'];
    $y2 = $data['value4'];
    $time = $data['reading_time'];
}

$result->free();
$conn->close();

header("Content-type: text/json");
//$ret = array((float)$x,(float)$y);
$ret = array('xaxis1'=>(float)$x1, 'yaxis1'=>(float)$y1, 'xaxis2'=>(float)$x2, 'yaxis2'=>(float)$y2); 
echo json_encode($ret);

?>