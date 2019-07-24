$(document).ready(function() {
    'use strict'

    var serverURL = "http://thegreenbot.local";
    // var serverURL = "http://localhost:8000";

    const wifiUrl = serverURL + '/api/wifi';
    const wifiStatusUrl = serverURL + '/api/wifi-status';
    const logsUrl = serverURL + '/api/logs';
    const eventsUrl = serverURL + '/api/events';
    const intelligenceUrl = serverURL + '/api/intelligence';
    const updateUrl = serverURL + '/api/update';


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
        url: updateUrl,
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