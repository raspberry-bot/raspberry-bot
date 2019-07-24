var serverURL = "http://thegreenbot.local";
// var serverURL = "http://localhost:8000";
const pingServerUrl = serverURL + '/api/ping';
const systemUrl = serverURL + '/api/system';


function SuccessFuncAfterNavBarLoaded(){

  $('#systemInfoButton').click( function() {
    let sysInfo = $('#sysinfolist');
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


$(document).ready(function() {
    'use strict'
    includeHTML();

    var pingInternal = 100;

    $('nav').hide();
    $('body').append('<div style="" class="loading text-center" id="loadingScreen">This page will referesh once the robot becomes online again...</div>');

    $(window).on('load', function(){
      setTimeout(pingServer, pingInternal);
    });
    
    function pingServer() {
      $.ajax({
          url: pingServerUrl + "?t=" + new Date().getTime(),
          type: 'get',
          cache: false,
          success: function(data){
            $("#loadingScreen").fadeOut(500, function() {
              // fadeOut complete. Remove the loading div
              $("#loadingScreen" ).hide(); //makes page more lightweight 
              $('nav').show();
            }); 
            setTimeout(pingServer, pingInternal);
          },
          error: function(){
              console.log('Error connecting to server!');
              $('#loadingScreen').show();
              $('nav').hide();
              setTimeout(pingServer, pingInternal);
          }
      });
    }
    pingServer();
});