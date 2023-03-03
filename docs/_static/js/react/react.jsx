const useState = React.useState;

// function getPathToResource(e, t) {
//   return [...new Array(e.page_nesting_level).fill(".."), t].join("/")
// }
// ! function a(n, o, i) {
//   function u(t, e) {
//       if (!o[t]) {
//           if (!n[t]) {
//               var r = "function" == typeof require && require;
//               if (!e && r) return r(t, !0);
//               if (c) return c(t, !0);
//               throw (e = new Error("Cannot find module '" + t + "'")).code = "MODULE_NOT_FOUND", e
//           }
//           r = o[t] = {
//               exports: {}
//           }, n[t][0].call(r.exports, function(e) {
//               return u(n[t][1][e] || e)
//           }, r, r.exports, a, n, o, i)
//       }
//       return o[t].exports
//   }
//   for (var c = "function" == typeof require && require, e = 0; e < i.length; e++) u(i[e]);
//   return u
// }({
//   1: [function(e, t, r) {
//       window.useState = React.useState, window.MUI = window.MaterialUI, window.getPathToResource = function(e, t) {
//           return [...new Array(e.page_nesting_level).fill(".."), t].join("/")
//       }, window.mountComponent = function(e, t) {
//           for (const n of document.querySelectorAll(e)) {
//               var r = function(e) {
//                   var t, r, n = {};
//                   for (const o of e.getAttributeNames()) o.startsWith("data-") && (t = o.slice(5), r = decodeURIComponent(e.getAttribute(o)), n[t] = "sphinx_state" === t ? JSON.parse(r) : r);
//                   return n
//               }(n);
//               ReactDOM.createRoot(n).render(React.createElement(t, r))
//           }
//       }
//   }, {}]
// }, {}, [1]);

function getComponentProps(node) {
  const attributeNames = node.getAttributeNames();
  const props = {};
  for (const attributeName of attributeNames) {
    if (attributeName.startsWith("data-")) {
      const propName = attributeName.slice(5);
      props[propName] = node.getAttribute(attributeName);
    }
  }
  return props;
}

function mountComponent(querySelector, Component) {
  const containers = document.querySelectorAll(querySelector);
  for (const container of containers) {
    const props = getComponentProps(container);
    const root = ReactDOM.createRoot(container);
    root.render(<Component {...props} />);
  }
}

// LikeButtonWithTitle Component

function LikeButtonWithTitle({ title, margin, padding }) {
  const [likeCount, setLikeCount] = useState(100500);
  return (
    <button onClick={() => setLikeCount(likeCount + 1)} style={{ margin, padding }}>
      Like {title} {likeCount}
    </button>
  );
}

mountComponent(".LikeButtonWithTitle", LikeButtonWithTitle);


// ReactGreeter component

function ReactGreeter() {
  const [name, setName] = useState("");
  const onSubmit = (event) => {
    event.preventDefault();
    alert(`Hello, ${name}!`);
  };
  return (
    <form onSubmit={onSubmit}>
      <input
        type="text"
        placeholder="Enter your name"
        required
        value={name}
        onChange={(event) => setName(event.target.value)}
      />
      <button type="submit" disabled={!name}>Submit</button>
    </form>
  );
}

mountComponent(".ReactGreeter", ReactGreeter);


(function () {
  const __NewReactComponent__ = (function () {
      const module = {};
      function AppCardDirective({ sphinx_state: a, app_id: b, image: c, title: d, description: e, tags: f, width: g, preview:
          h, target: i }) {
              const [j, k] = useState("hidden-details"), l = .75 * parseInt(g);
          return/*#__PURE__*/React.createElement("div", {
              onClick: () => window.open(i, "_blank"), className: "lit-card", style: {
                  width: g + "px", height: l + "px"
              }, onMouseEnter: () => k("visible-details"), onMouseLeave: () => k("hidden-details")
          },/*#__PURE__*/React.createElement("div", { className: "header" },/*#__PURE__*/React.createElement("img", {
              style: {
                  width: "100%", height: "100%", objectFit: "cover", borderRadius: "6px"
              }, src: c
          }),/*#__PURE__*/React.createElement("div", { className: `app-card-cover + ${j}` })), d === void 0 ? null
              :/*#__PURE__*/React.createElement("div", { className: "app-title-holder" },/*#__PURE__*/React.createElement("div", {
                  className: "app-title"
              }, d),/*#__PURE__*/React.createElement("div", { className: "app-tag" }, f)), e !== void 0 &&
                  "visible-details" !== j ?/*#__PURE__*/React.createElement("div", { className: "app-description" }, e) : null, h == null
                      && b == null ? null :/*#__PURE__*/React.createElement("div", {
                          className: "footer"
                      },/*#__PURE__*/React.createElement("button", {
                          className: "secondary-button", onClick: a => {
                              a.stopPropagation(),
                              window.open(h, "_blank")
                          }, style: { opacity: h == null ? 0 : 1 }
                      },/*#__PURE__*/React.createElement("img", {
                          className:
                              "play-button", style: { filter: "brightness(01) invert(0.5)" }, src: window.getPathToResource(a,
                                  "_static/images/icon-view.svg")
                      }), "Preview"),/*#__PURE__*/React.createElement("button", {
                          className: "primary-button",
                          onClick: a => { a.stopPropagation(), window.LAI.runApp(b) }, style: b ? void 0 : { display: "none" }
                      },/*#__PURE__*/React.createElement("img", {
                          class: "play-button", src: window.getPathToResource(a,
                              "_static/images/icon-launch.svg")
                      }), "Run")))
      }// Export your Directive component using `module.exports = `
      module.exports = AppCardDirective;
      return module.exports;
  })();
  mountComponent('.AppCardDirective', __NewReactComponent__);
})();
