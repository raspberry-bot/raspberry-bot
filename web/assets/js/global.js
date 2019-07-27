var serverURL = "http://thegreenbot.local";
// var serverURL = "http://localhost:8000";
const systemUrl = serverURL + '/api/system';


function SuccessFuncAfterNavBarLoaded(){

  $('#systemInfoButton').click( function() {
    let sysInfo = $('#sysinfolist');
    sysInfo.empty();
    $.getJSON(systemUrl, function (data) {
      data.forEach(function (item) {
        sysInfo.append($('<li>' + item + '</li>'));
      })
    });
  });

  $('#shutdownButton').click( function() {
    console.log('shutdownButton is clicked...')
    $.ajax({
        url: systemUrl,
        type: 'post',
        dataType: 'json',
        data: JSON.stringify({"command": "shutdown"}),
        success: function(data) {
          console.log('Shutting Down...')
        }
    });
  });


  $('#rebootButton').click( function() {
    console.log('rebootButton is clicked...')
    $.ajax({
        url: systemUrl,
        type: 'post',
        dataType: 'json',
        data: JSON.stringify({"command": "reboot"}),
        success: function(data) {
          console.log('Rebooting...')
        }
    });
  });
  feather.replace();
}

function includeHTML() {
    var z, i, elmnt, file, xhttp;
    /*loop through a collection of all HTML elements:*/
    z = document.getElementsByTagName("*");
    for (i = 0; i < z.length; i++) {
      elmnt = z[i];
      /*search for elements with a certain atrribute:*/
      file = elmnt.getAttribute("include-html");
      if (file) {
          /*make an HTTP request using the attribute value as the file name:*/
          xhttp = new XMLHttpRequest();
          xhttp.onreadystatechange = function() {
            if (this.readyState == 4) {
                if (this.status == 200) {
                  elmnt.innerHTML = this.responseText;
                  SuccessFuncAfterNavBarLoaded();
                }
                if (this.status == 404) {elmnt.innerHTML = "Page not found.";}
                /*remove the attribute, and call this function once more:*/
                elmnt.removeAttribute("include-html");
                includeHTML();
            }
          }
          xhttp.open("GET", file, true);
          xhttp.send();
          /*exit the function:*/
          return;
      }
    }
};

function setupWebSocket() {
  var wsProtocol = (location.protocol === "https:") ? "wss://" : "ws://";

  var wsPing = new WebSocket(wsProtocol + "thegreenbot.local" + "/api/ping");
  // ws.binaryType = 'arraybuffer';

  wsPing.onopen = function() {
      console.log("connection was established for Ping");
      if (wsPing != null && wsPing.readyState === WebSocket.OPEN) {
        wsPing.send(1)
      }
  };
  
  wsPing.onmessage = function(evt) {
      var message = evt.data;
      // console.log('Last Ping Timestamp: ' + message)
      $("#loadingScreen").fadeOut(500, function() {
        $("#loadingScreen" ).hide(); //makes page more lightweight 
        $('nav').show();
      });
  };

  wsPing.onerror = function (e) {
      console.log(e);
      console.log('Error connecting to server!');
      $('#loadingScreen').show();
      $('nav').hide();
  };

  wsPing.onclose = function (e) {
      console.log(e);
      console.log('Closing connection to server!');
      $('#loadingScreen').show();
      $('nav').hide();
      setTimeout(setupWebSocket, 1000);
  };
  return wsPing;
}
var wsPing;
$(document).ready(function() {
  'use strict'
  includeHTML();

  $('nav').hide();
  $('body').append('<div style="" class="loading text-center" id="loadingScreen">This page will referesh once the robot becomes online again...</div>');

  if ("WebSocket" in window) {
    wsPing = setupWebSocket();
    if (wsPing != null && wsPing.readyState === WebSocket.OPEN) {
      wsPing.send(1);
    }
  } else {
    alert("WebSocket not supported");
  }
});