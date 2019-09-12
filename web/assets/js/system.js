var serverURL = "http://raspberrybot.local";
// var serverURL = "http://localhost:8000";

const logsUrl = serverURL + '/api/logs';
const eventsUrl = serverURL + '/api/events';
const servicesUrl = serverURL + '/api/system/services';
const updateUrl = serverURL + '/api/update';

function getFormDataInJson(form) {
  var object = {};
  form.forEach((item) => { object[item.name] = item.value });
  return JSON.stringify(object);
}
function getToggleFormDataInJson(form) {
  var object = {};
  form.forEach((item) => { object[item.name] = item.value });
  return JSON.stringify(object);
}

function populateLogs() {
  $.getJSON(logsUrl + '?t=' + new Date().getTime(), function (data) {
    document.querySelector("#syslogTabContent").innerHTML = data.syslog;
    document.querySelector("#eventsTabContent").innerHTML = data.events;
    document.querySelector("#supervisordTabContent").innerHTML = data.supervisord;
    document.querySelector("#tornadoTabContent").innerHTML = data.tornado;
    document.querySelector("#jupyterTabContent").innerHTML = data.jupyter;
    document.querySelector("#nginxTabContent").innerHTML = data.nginx;
  });
}

$(document).ready(function () {
  'use strict'
  populateLogs();
  $('#refreshLogs').click(populateLogs);
  // Populate Services Toggle Form
  $.getJSON(servicesUrl, function (data) {
    // <div class="form-group"><div class="checkbox"><label><input class="form-control" id="service_x" name="voice-command" type="checkbox" data-toggle="toggle" data-on="Enabled" data-off="Disabled" data-onstyle="success" data-offstyle="danger">Service_X</label></div></div>
    var serviceslist = $('serviceslist');
    console.log(data);
    for (var service in data) {
      var newFormGroupOpen = '<div class="form-group"><div class="checkbox"><label>';
      var newFormGroupClose = '</label></div></div>';
      var newService = $('<input class="form-control" type="checkbox" data-toggle="toggle" data-on="Enabled" data-off="Disabled" data-onstyle="success" data-offstyle="danger">')
        .attr('id', service.name)
        .attr('name', service.name)
        .attr('value', service.name);

      var formGroup = newFormGroupOpen + newService + newFormGroupClose;

      if (service.statename == 'RUNNING') {
        formGroup.bootstrapToggle('on');
      } else {
        formGroup.bootstrapToggle('off');
      }
      serviceslist.appendChild(formGroup);
    }
  });

  $("#servicesForm").submit(function (e) {
    e.preventDefault();
  });

  $('#servicesSaveButton').click(function () {
    console.log('servicesSaveButton is clicked...')
    $.ajax({
      url: servicesUrl,
      type: 'post',
      dataType: 'json',
      data: getToggleFormDataInJson($('form#servicesForm').serializeArray()),
      success: function (data) {
        console.log('Saving Services Modules Configuration...')
      }
    });
  });


  function countdown(remaining) {
    if (remaining <= 0)
      location.reload(true);
    document.querySelector("#updateInfo").innerHTML = "Updating... \n Page will be refreshed after " + remaining + " seconds...";
    setTimeout(function () { countdown(remaining - 1); }, 1000);
  }

  $("#updateForm").submit(function (e) {
    e.preventDefault();
    countdown(30);
  });

  $('#updateButton').click(function () {
    console.log('connectButton is clicked...')
    let formDataInJson = getFormDataInJson($('form#updateForm').serializeArray());
    $.ajax({
      url: updateUrl,
      type: 'post',
      dataType: 'json',
      data: JSON.stringify(formDataInJson),
      success: function (data) {
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