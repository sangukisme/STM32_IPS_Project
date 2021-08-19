<?php 
    header("Content-Type: text/html;charset=UTF-8"); 
    
    $host = 'localhost'; 
    $user = 'root';
    $pw = 'park1234'; 
    $dbName = 'esp_data'; 
    $mysqli = new mysqli($host, $user, $pw, $dbName); 
    
      if($mysqli){ 
          //echo "MySQL successfully connected!<br/>"; 
          
          $num1 = $_GET['num1']; 
          $num2 = $_GET['num2']; 
          $num3 = $_GET['num3']; 
          $num4 = $_GET['num4']; 
          
          //echo "<br/>num1 = $num1"; 
          //echo ", "; 
          //echo "num2 = $num2<br/>"; 
          
          $query = "INSERT INTO sensor2 (value1, value2, value3, value4) VALUES ('$num1','$num2','$num3','$num4')";
          mysqli_query($mysqli,$query); 
          //echo "</br>success!!"; 
      } 
      
      else{ 
          //echo "MySQL could not be connected"; 
      } 
     
     //echo "<br/>$num1 success<br/>";
     mysqli_close($mysqli); 
?>
