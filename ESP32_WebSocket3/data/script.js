window.addEventListener('load', getValues);

function getValues() {
    var xhr = new XMLHttpRequest();
    xhr.onreadystatechange = function() {
        if(this.readyState == 4 && this.status == 200) {
            var myObj = JSON.parse(this.responseText);
            console.log(myObj);
            document.getElementById("textFieldValue").innerHTML = myObj.textValue;
            document.getElementById("numberFieldValue").innerHTML = myObj.numberValue;
            document.getElementById("thetaFieldValue").innerHTML = myObj.thetaValue;
            document.getElementById("xrFieldValue").innerHTML = myObj.xrValue;
            document.getElementById("yrFieldValue").innerHTML = myObj.yrValue;
        }
    };

    xhr.open("GET", "/values", true);
    xhr.send();
}