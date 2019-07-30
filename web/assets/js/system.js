var serverURL = "http://thegreenbot.local";
// var serverURL = "http://localhost:8000";

const logsUrl = serverURL + '/api/logs';
const eventsUrl = serverURL + '/api/events';
const intelligenceUrl = serverURL + '/api/intelligence';
const updateUrl = serverURL + '/api/update';

function getFormDataInJson(form){
  var object = {};
  form.forEach((item) => {object[item.name] = item.value});
  return JSON.stringify(object);
}
function getToggleFormDataInJson(form){
  var object = {};
  form.forEach((item) => {object[item.name] = item.value});
  return JSON.stringify(object);
}

function populateLogs(){
  $.getJSON(logsUrl, function (data) {
    document.querySelector("#syslogTabContent").innerHTML = data.syslog;
    document.querySelector("#eventsTabContent").innerHTML = data.events;
    document.querySelector("#supervisordTabContent").innerHTML = data.supervisord;
    document.querySelector("#tornadoTabContent").innerHTML = data.tornado;
    document.querySelector("#jupyterTabContent").innerHTML = data.jupyter;
    document.querySelector("#nginxTabContent").innerHTML = data.nginx;
  });
}

$(document).ready(function() {
  'use strict'
  populateLogs();
  $('#refreshLogs').click(populateLogs);
  // Populate Intelligence Toggle Form
  // $.getJSON(intelligenceUrl, function (data) {
  //   for (var item in data) {
  //     let toggle = $("#intelligence-" + item);
  //     if (data[item] == true) {
  //       toggle.bootstrapToggle('on');
  //     } else {
  //       toggle.bootstrapToggle('off');
  //     }
      
  //   }
  // });

  // $("#intelligenceForm").submit(function(e) {
  //   e.preventDefault();
  // });

  // $('#intelligenceSaveButton').click( function() {
  //   console.log('intelligenceSaveButton is clicked...')
  //   $.ajax({
  //       url: intelligenceUrl,
  //       type: 'post',
  //       dataType: 'json',
  //       data: getToggleFormDataInJson($('form#intelligenceForm').serializeArray()),
  //       success: function(data) {
  //         console.log('Saving Intelligence Modules Configuration...')
  //       }
  //   });
  // });

  
  function countdown(remaining) {
    if(remaining <= 0)
        location.reload(true);
        document.querySelector("#updateInfo").innerHTML = "Updating... \n Page will be refreshed after " + remaining + " seconds...";
    setTimeout(function(){ countdown(remaining - 1); }, 1000);
  }

  $("#updateForm").submit(function(e) {
    e.preventDefault();
    countdown(30);
  });

  $('#updateButton').click( function() {
    console.log('connectButton is clicked...')
    let formDataInJson = getFormDataInJson($('form#updateForm').serializeArray());
    $.ajax({
        url: updateUrl,
        type: 'post',
        dataType: 'json',
        data: JSON.stringify(formDataInJson),
        success: function(data) {
          console.log('Getting response back from git...')
          debugger;
          $('#gitresult').val(data);
        }
    });
  });

  $.getJSON(updateUrl, function (data) {
    var msg;
    var versionLocal = '(Your System Version: ' + data.firmware.version + ')';
    var versionRemote = '(Latest Version: ' + data.latest_version + ')';
    if (data.new_update_available == true) {
      msg = 'New Update Is Available!' + versionLocal + ' < ' + versionRemote;
      // TODO: Add a help tooltip here :
      // if you are using a forked repo make sure to have a /VERSION file in the root of your repo with a float value higher than your current firmware version.
      $('#updateButton').prop('enabled', true);
    } else {
      msg = 'Your system is up to date! ' + versionLocal + ' == ' + versionRemote;
      $('#updateButton').prop('disabled', true);
    }
    document.querySelector("#updateInfo").innerHTML = msg;
  });

});