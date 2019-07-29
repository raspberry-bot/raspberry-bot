var serverURL = "http://thegreenbot.local";
// var serverURL = "http://localhost:8000";
const systemUrl = serverURL + '/api/system';
const wifiUrl = serverURL + '/api/wifi';
const wifiStatusUrl = serverURL + '/api/wifi-status';

function getFormDataInJson(form){
  var object = {};
  form.forEach((item) => {object[item.name] = item.value});
  return object;
}

var wifiSpinner = "<div class=\"d-flex justify-content-center\" id=\"ssid-search-status\"><div class=\"spinner-border text-success\" role=\"status\"><span class=\"sr-only\">Loading...</span></div></div>"

function SuccessFuncAfterNavBarLoaded(){

    // Populate dropdown with list of Wifis
    function populateWifiList(){
      var spinner = document.getElementById("wifiSpinner");
      spinner.innerHTML = wifiSpinner;

      let wifiConnectionList = $('#wifiConnectionList');
      wifiConnectionList.empty();
      $.getJSON(wifiUrl, function (data) {
        $.each(data, function (key, entry) {
          if (key.length > 0) {
            var newItem = $('<a class="dropdown-item ssidConnect" data-toggle="modal" data-target="#wifiConnectModal"></a>')
            .attr('data-id', key)
            .attr('id', key)
            .attr('value', key);
            if (entry == 'connected'){
              newItem.text(key);
              newItem.attr('style', "color:green; background-color:white;");
              newItem.prepend("<span data-feather=\"wifi\"></span>&nbsp;");
            } else if (entry == 'disconnect'){
              newItem.text(key);
              newItem.prepend("<span data-feather=\"wifi-off\"></span>&nbsp;");
            }
            wifiConnectionList.append(newItem);
          }
        });
        if (spinner != null){
          spinner.innerHTML = "";
        };
        feather.replace();
      });
    };

    $("#wifidropdown").click(populateWifiList);

    $(document).on("click", ".ssidConnect", function () {
      $("#selected-ssid").val($(this).data('id'));
    });

    function populateWifiConnectionInfo(){
      $.getJSON(wifiStatusUrl, function (data) {
        document.getElementById("wifiConnectionInfo").innerHTML = data;
      });
    }
    $('#wifiConnectInfoButton').click(populateWifiConnectionInfo);

  
    $("#wifiForm").submit(function(e) {
      e.preventDefault();
    });

    function countdown(remaining) {
      if(remaining <= 0)
          location.reload(true);
          document.getElementById("connectionResult").innerHTML = "Page will be refreshed after " + remaining + " seconds...";
      setTimeout(function(){ countdown(remaining - 1); }, 1000);
    }

    $('#connectButton').click( function() {
      var jsonData = getFormDataInJson($('form#wifiForm').serializeArray());
      document.getElementById("connectionResult").innerHTML = 'Connecting to: ' + jsonData["selected-ssid"];
      $.ajax({
          url: wifiUrl,
          type: 'post',
          dataType: 'json',
          data: JSON.stringify(jsonData),
          success: function(response) {
            $("#connectionResult").innerHTML = response;
            document.getElementById("connectionResult").innerHTML = "Successfully Connected To: " + jsonData["selected-ssid"]
          },
          error: function(xhr, ajaxOptions, thrownError) {
            // console.log(thrownError);
            document.getElementById("connectionResult").innerHTML = thrownError;
          }
      });
      countdown(120);
    });

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
                  feather.replace();
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