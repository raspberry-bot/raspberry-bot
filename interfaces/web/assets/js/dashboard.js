/* globals Chart:false, feather:false */

(function($) {
  $(document).ready(function() {
    'use strict'

    feather.replace()

    var serverURL = "http://thegreenbot.local";
    // var serverURL = "http://localhost:8000";

    const wifiUrl = serverURL + '/api/wifi';
    const wifiStatusUrl = serverURL + '/api/wifi-status';
    const systemUrl = serverURL + '/api/system';
    const logsUrl = serverURL + '/api/logs';
    const eventsUrl = serverURL + '/api/events';
    const intelligenceUrl = serverURL + '/api/intelligence';

    $.getJSON(wifiStatusUrl, function (data) {
      $('#wifistatus').val(data)
    });
  
    // Populate Logs
    $.getJSON(logsUrl, function (data) {
      $('#logsview').val(data);
    });

    $.getJSON(eventsUrl, function (data) {
      $('#eventsview').val(data);
    });

    // Populate System Info
    let sysInfo = $('#sysinfolist');

    $.getJSON(systemUrl, function (data) {
      data.forEach(function (item) {
        sysInfo.append($('<li>' + item + '</li>'));
      })
    });

  // Populate dropdown with list of Wifis
  let dropdown = $('#ssid');

  dropdown.empty();

  dropdown.append('<option selected="true" disabled>Choose Your Wifi Network</option>');
  dropdown.prop('selectedIndex', 0);
  $.getJSON(wifiUrl, function (data) {
    $.each(data, function (key, entry) {
      dropdown.append($('<option></option>').attr('value', entry).text(entry));
    })
  });

  $("#wifiForm").submit(function(e) {
    e.preventDefault();
  });

  function getFormDataInJson(form){
    var object = {};
    form.forEach((item) => {object[item.name] = item.value});
    return JSON.stringify(object);
  }


  $('#connectButton').click( function() {
    console.log('connectButton is clicked...')
    $.ajax({
        url: wifiUrl,
        type: 'post',
        dataType: 'json',
        data: getFormDataInJson($('form#wifiForm').serializeArray()),
        success: function(data) {
          console.log('Connecting to Wifi...')
        }
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


  $('#resetFactoryButton').click( function() {
    console.log('resetFactoryButton is clicked...')
    $.ajax({
        url: systemUrl,
        type: 'post',
        dataType: 'json',
        data: JSON.stringify({"command": "reset_factory"}),
        success: function(data) {
          console.log('Reset Factory...')
        }
    });
  });


  // Populate Intelligence Toggle Form
  $.getJSON(intelligenceUrl, function (data) {
    for (var item in data) {
      let toggle = $("#intelligence-" + item);
      if (data[item] == true) {
        toggle.bootstrapToggle('on');
      } else {
        toggle.bootstrapToggle('off');
      }
      
    }
  });

  $("#intelligenceForm").submit(function(e) {
    e.preventDefault();
  });



  function getToggleFormDataInJson(form){
    var object = {};
    form.forEach((item) => {object[item.name] = item.value});
    return JSON.stringify(object);
  }


  $('#intelligenceSaveButton').click( function() {
    console.log('intelligenceSaveButton is clicked...')
    $.ajax({
        url: intelligenceUrl,
        type: 'post',
        dataType: 'json',
        data: getToggleFormDataInJson($('form#intelligenceForm').serializeArray()),
        success: function(data) {
          console.log('Saving Intelligence Modules Configuration...')
        }
    });
  });

  $("#updateForm").submit(function(e) {
    e.preventDefault();
  });

  $('#updateButton').click( function() {
    console.log('connectButton is clicked...')
    $.ajax({
        url: wifiUrl,
        type: 'post',
        dataType: 'json',
        data: getFormDataInJson($('form#updateForm').serializeArray()),
        success: function(data) {
          console.log('Getting response back from git...')
          $('#gitresult').val(data);
        }
    });
  });

  });
})($);
