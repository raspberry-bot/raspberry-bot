var serverURL = "http://raspberrybot.local";
// var serverURL = "http://localhost:8000";
var wsPing;


const systemUrl = serverURL + '/api/system';
const wifiUrl = serverURL + '/api/wifi';
const wifiAccessPointSetupUrl = serverURL + '/api/wifi-access-point';
const wifiStatusUrl = serverURL + '/api/wifi-status';

function getFormDataInJson(form) {
  var object = {};
  form.forEach((item) => { object[item.name] = item.value });
  return object;
}

function countdownConnectionResult(remaining) {
  if (remaining <= 0)
    location.reload(true);
  document.getElementById("connectionResult").innerHTML = "This page will be refreshed after " + remaining + " seconds...";
  setTimeout(function () { countdownConnectionResult(remaining - 1); }, 1000);
}

function countdownAccessPointResult(remaining) {
  if (remaining <= 0)
    location.reload(true);
  document.getElementById("accessPointResultRefresh").innerHTML = "This page will be refreshed after " + remaining + " seconds...";
  setTimeout(function () { countdownAccessPointResult(remaining - 1); }, 1000);
}

var wifiSpinner = "<div class=\"d-flex justify-content-center\" id=\"ssid-search-status\"><div class=\"spinner-border text-success\" role=\"status\"><span class=\"sr-only\">Loading...</span></div></div>"

var markMenuItemAsActive = function () {
  // get current URL path and assign 'active' class
  // Get current page URL
  var url = window.location.href;
  // remove # from URL
  url = url.substring(0, (url.indexOf("#") == -1) ? url.length : url.indexOf("#"));
  // remove parameters from URL
  url = url.substring(0, (url.indexOf("?") == -1) ? url.length : url.indexOf("?"));

  // select file name
  url = url.substr(url.lastIndexOf("/") + 1);
  // Loop all menu items
  $('.navbar-nav li').each(function () {
    // select href
    var href = $(this).find('a').attr('href');
    // Check filename
    if (url == href) {
      // Add active class
      $(this).addClass('active');
    }
  });
}

function SuccessFuncAfterNavBarLoaded() {

  // Populate dropdown with list of Wifis
  function populateWifiList() {
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
          if (entry == 'connected') {
            newItem.text(key);
            newItem.attr('style', "color:green; background-color:white;");
            newItem.prepend("<span data-feather=\"wifi\"></span>&nbsp;");
            wifiConnectionList.prepend(newItem);
          } else if (entry == 'disconnect') {
            newItem.text(key);
            newItem.prepend("<span data-feather=\"wifi-off\"></span>&nbsp;");
            wifiConnectionList.append(newItem);
          }
        }
      });
      if (spinner != null) {
        spinner.innerHTML = "";
      };

      feather.replace();
    });
  };

  $("#wifidropdown").click(populateWifiList);

  $(document).on("click", ".ssidConnect", function () {
    $("#selected-ssid").val($(this).data('id'));
  });

  function populateWifiConnectionInfo() {
    $.getJSON(wifiStatusUrl, function (data) {
      document.getElementById("wifiConnectionInfo").innerHTML = data;
    });
  }
  $('#wifiConnectInfoButton').click(populateWifiConnectionInfo);


  $("#wifiForm").submit(function (e) {
    e.preventDefault();
  });

  $('#connectButton').click(function () {
    var jsonData = getFormDataInJson($('form#wifiForm').serializeArray());
    document.getElementById("connectionResult").innerHTML = 'Connecting to: ' + jsonData["selected-ssid"];
    $.ajax({
      url: wifiUrl,
      type: 'post',
      dataType: 'json',
      data: JSON.stringify(jsonData),
      success: function (response) {
        // $("#connectionResult").innerHTML = response;
        document.getElementById("connectionResult").innerHTML = "Successfully Connected To: " + jsonData["selected-ssid"]
      },
      error: function (xhr, ajaxOptions, thrownError) {
        // console.log(thrownError);
        document.getElementById("connectionResult").innerHTML = thrownError;
      }
    });
    countdownConnectionResult(40);
  });

  $("#wifiAccessPointForm").submit(function (e) {
    e.preventDefault();
  });

  $('#setupAccessPointButton').click(function () {
    $.ajax({
      url: wifiAccessPointSetupUrl,
      type: 'post',
      data: JSON.stringify({}),
      error: function (xhr, ajaxOptions, thrownError) {
        console.log(thrownError);
        document.getElementById("accessPointResult").innerHTML = thrownError;
      }
    });
    // countdownAccessPointResult(40);
  });

  $('#systemInfoButton').click(function () {
    $.getJSON(systemUrl, function (data) {
      document.querySelector("#sysInfo").innerHTML = data;
    });
  });

  $('#shutdownButton').click(function () {
    console.log('shutdownButton is clicked...')
    $.ajax({
      url: systemUrl,
      type: 'post',
      dataType: 'json',
      data: JSON.stringify({ "command": "shutdown" }),
      success: function (data) {
        console.log('Shutting Down...')
      }
    });
  });


  $('#rebootButton').click(function () {
    console.log('rebootButton is clicked...')
    $.ajax({
      url: systemUrl,
      type: 'post',
      dataType: 'json',
      data: JSON.stringify({ "command": "reboot" }),
      success: function (data) {
        console.log('Rebooting...')
      }
    });
  });

  markMenuItemAsActive();
};

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
      xhttp.onreadystatechange = function () {
        if (this.readyState == 4) {
          if (this.status == 200) {
            elmnt.innerHTML = this.responseText;
            SuccessFuncAfterNavBarLoaded();
            feather.replace();
          }
          if (this.status == 404) { elmnt.innerHTML = "Page not found."; }
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
}

function setupWebSocket() {
  var wsProtocol = (location.protocol === "https:") ? "wss://" : "ws://";

  var wsPing = new WebSocket(wsProtocol + "raspberrybot.local" + "/api/ping");
  // ws.binaryType = 'arraybuffer';

  wsPing.onopen = function () {
    console.log("connection was established for Ping");
    if (wsPing != null && wsPing.readyState === WebSocket.OPEN) {
      wsPing.send(1)
    }
  };

  wsPing.onmessage = function (evt) {
    var message = evt.data;
    // console.log('Last Ping Timestamp: ' + message)
    $("#loadingScreen").fadeOut(500, function () {
      $("#loadingScreen").hide(); //makes page more lightweight 
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

$(document).ready(function () {
  'use strict'
  includeHTML();

  markMenuItemAsActive();
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
  $('.nav-tabs a:first').tab('show');

  if (userAgent.isIos()) {
    document.querySelector('html').classList.add('is-ios');
  }
});