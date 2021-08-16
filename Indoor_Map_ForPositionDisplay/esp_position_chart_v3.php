<!DOCTYPE HTML><html>
<!-- Rui Santos - Complete project details at https://RandomNerdTutorials.com

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files.
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software. -->
<head>
    <meta http-equiv="Content-Type" content="text/html; charset=utf-8">
    <title>Highcharts Example</title>

    <script type="text/javascript" src="http://code.jquery.com/jquery-1.10.1.js"></script>
    <script type="text/javascript" src="http://code.highcharts.com/highcharts.js"></script>

</head>
<body>
    <div id="container1" style="width: 800px; height: 800px; margin: 0 auto"></div>
    <div id="container2" style="width: 800px; height: 800px; margin: 0 auto"></div>
</body>
<script>

var chart1; // global
var chart2; // global
     		
function requestData() {
    $.ajax({
        type: 'POST',
    	url: 'proc2.php',
        dataType: 'json',
    	success: function(point) {
            //var value = JSON.parse('point'); 
    		var x1 = point.xaxis1,
                y1 = point.yaxis1,
                x2 = point.xaxis2,
                y2 = point.yaxis2,
                series1 = chart1.series[0],
                series2 = chart2.series[0];
    			//shift = series.data.length > 20; // shift if the series is longer than 20
    	
    		//chart.series[0].addPoint(eval(point), true, shift);
    		//chart.series[0].addPoint(eval(point), true);
    		chart1.series[0].addPoint([x1, y1]);
    		chart2.series[0].addPoint([x2, y2]);
    					
    		// call it again after 50ms
    		setTimeout(requestData, 50);	
    	},
    	cache: false
    });
}
    			
$(document).ready(function() {
    chart1 = new Highcharts.Chart({
    	chart: {
            type : 'scatter',
            margin: [70, 70, 80, 80],
    		renderTo: 'container1',
    		//defaultSeriesType: 'spline',
    		events: {
    			load: requestData
    		}
    	},
    	title: {
    		text: 'Indoor Position'
    	},
        accessibility: {
            announceNewData: {
                enabled: true
            }
        },
    	xAxis: {
            title: {
                text: 'X-Axis'
            },
            //min:-100,
            //max:100,
            gridLineWidth: 1,
            minPadding: 0.2,
            maxPadding: 0.2,
            maxZoom: 60
        },
        yAxis: {
            title: {
                text: 'y-Axis'
            },
            //min:-100,
            //max:100,
            minPadding: 0.2,
            maxPadding: 0.2,
            maxZoom: 60,
            plotLines: [{
                value: 0,
                width: 1,
                color: '#808080'
            }]
        },
        legend: {
            enabled: false
        },
        exporting: {
            enabled: false
        },
        plotOptions: {
            series: {
                lineWidth: 1,
                marker: {
                    enabled: true        
				}
			}  
		},
    	series: [{			
    		name: 'encoder_Position',
    		data: []
    	}]
    });

    chart2 = new Highcharts.Chart({
    	chart: {
            type : 'scatter',
            margin: [70, 70, 80, 80],
    		renderTo: 'container2',
    		//defaultSeriesType: 'spline',
    		events: {
    			load: requestData
    		}
    	},
    	title: {
    		text: 'Indoor Position'
    	},
        accessibility: {
            announceNewData: {
                enabled: true
            }
        },
    	xAxis: {
            title: {
                text: 'X-Axis'
            },
            //min:-100,
            //max:100,
            gridLineWidth: 1,
            minPadding: 0.2,
            maxPadding: 0.2,
            maxZoom: 60
        },
        yAxis: {
            title: {
                text: 'y-Axis'
            },
            //min:-100,
            //max:100,
            minPadding: 0.2,
            maxPadding: 0.2,
            maxZoom: 60,
            plotLines: [{
                value: 0,
                width: 1,
                color: '#808080'
            }]
        },
        legend: {
            enabled: false
        },
        exporting: {
            enabled: false
        },
        plotOptions: {
            series: {
                lineWidth: 1,
                marker: {
                    enabled: true        
				}
			}  
		},
    	series: [{			
    		name: 'Gyro_position',
    		data: []
    	}]
    });
});

</script>
</html>
