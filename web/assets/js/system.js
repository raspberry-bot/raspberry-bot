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

$(document).ready(function() {
  'use strict'

  var serverURL = "http://thegreenbot.local";
  // var serverURL = "http://localhost:8000";

  const logsUrl = serverURL + '/api/logs';
  const eventsUrl = serverURL + '/api/events';
  const intelligenceUrl = serverURL + '/api/intelligence';
  const updateUrl = serverURL + '/api/update';

  // Populate Logs
  $.getJSON(logsUrl, function (data) {
    document.querySelector("#logsview").innerHTML = data;
  });

  $.getJSON(eventsUrl, function (data) {
    document.querySelector("#eventsview").innerHTML = data;
  });


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

  $.getJSON(updateUrl, function (data) {
    debugger;
    document.querySelector("#eventsview").innerHTML = data;
  });

});