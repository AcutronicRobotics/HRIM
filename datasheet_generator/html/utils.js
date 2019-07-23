function findObjectByKey(array, key, value) {
    for (var i = 0; i < array.length; i++) {
      // console.log(array[i].getAttribute(key))
        if (array[i].getAttribute(key) === value) {
            return array[i];
        }
    }
    return null;
}

var createXMLFile = (function () {
    var a = document.createElement("a");
    document.body.appendChild(a);
    a.style = "display: none";
    return function (data, name) {
        var blob = new Blob(data, {type: "text/xml"}),
            url = window.URL.createObjectURL(blob);
        a.href = url;
        a.download = name;
        a.click();
        window.URL.revokeObjectURL(url);
    };
}());
