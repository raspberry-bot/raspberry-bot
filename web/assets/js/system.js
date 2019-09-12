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
    // <div class="form-group">
    //  <div class="checkbox">
    //    <label><input class="form-control" id="service_x" name="voice-command" type="checkbox" data-toggle="toggle" data-on="Enabled" data-off="Disabled" data-onstyle="success" data-offstyle="danger">Service_X</label>
    //  </div>
    // </div>

    console.log(data);
    $.each(data,
      function (key, service) {
        var serviceslist = document.getElementById('serviceslist');
        console.log(service);
        var formGroup = document.createElement('div');
        formGroup.setAttribute('class', 'form-group');

        var checkbox = document.createElement('div');
        checkbox.setAttribute('class', 'checkbox');

        var label = document.createElement('label');

        var newService = document.createElement('input');
        newService.setAttribute('id', service.name);
        newService.setAttribute('type', 'checkbox')
        newService.setAttribute('data-toggle', 'toggle')
        newService.setAttribute('data-on', 'Enabled')
        newService.setAttribute('data-off', 'Disabled')
        newService.setAttribute('data-onstyle', 'success')
        newService.setAttribute('data-offstyle', 'danger')
        newService.setAttribute('class', 'form-control')
        newService.setAttribute('name', service.name)
        newService.setAttribute('value', service.name);


        label.appendChild(newService);
        label.innerHTML += service.name;

        checkbox.appendChild(label);
        formGroup.appendChild(checkbox);
        serviceslist.appendChild(formGroup);

        if (service.statename == 'RUNNING') {
          $('#' + service.name).bootstrapToggle('on');
        } else {
          $('#' + service.name).bootstrapToggle('off');
        }

      }
    );
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