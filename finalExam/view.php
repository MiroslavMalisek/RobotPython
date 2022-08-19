<!DOCTYPE html>

<html  dir="ltr" lang="es" xml:lang="es">
<head>
    <title>RobotPercComp-GII: Fichero .world del segundo recorrido</title>
    <link rel="shortcut icon" href="https://moodle.upm.es/titulaciones/oficiales/theme/image.php/boost/theme/1623142191/favicon" />
    <meta http-equiv="Content-Type" content="text/html; charset=utf-8" />
<meta name="keywords" content="moodle, RobotPercComp-GII: Fichero .world del segundo recorrido" />
<link rel="stylesheet" type="text/css" href="https://moodle.upm.es/titulaciones/oficiales/theme/yui_combo.php?rollup/3.17.2/yui-moodlesimple-min.css" /><script id="firstthemesheet" type="text/css">/** Required in order to fix style inclusion problems in IE with YUI **/</script><link rel="stylesheet" type="text/css" href="https://moodle.upm.es/titulaciones/oficiales/theme/styles.php/boost/1623142191_1617859296/all" />
<script>
//<![CDATA[
var M = {}; M.yui = {};
M.pageloadstarttime = new Date();
M.cfg = {"wwwroot":"https:\/\/moodle.upm.es\/titulaciones\/oficiales","sesskey":"FjPb0IgJ50","sessiontimeout":"7200","themerev":"1623142191","slasharguments":1,"theme":"boost","iconsystemmodule":"core\/icon_system_fontawesome","jsrev":"1623142191","admin":"admin","svgicons":true,"usertimezone":"Europa\/Madrid","contextid":8143720,"langrev":1623142191,"templaterev":"1623142191"};var yui1ConfigFn = function(me) {if(/-skin|reset|fonts|grids|base/.test(me.name)){me.type='css';me.path=me.path.replace(/\.js/,'.css');me.path=me.path.replace(/\/yui2-skin/,'/assets/skins/sam/yui2-skin')}};
var yui2ConfigFn = function(me) {var parts=me.name.replace(/^moodle-/,'').split('-'),component=parts.shift(),module=parts[0],min='-min';if(/-(skin|core)$/.test(me.name)){parts.pop();me.type='css';min=''}
if(module){var filename=parts.join('-');me.path=component+'/'+module+'/'+filename+min+'.'+me.type}else{me.path=component+'/'+component+'.'+me.type}};
YUI_config = {"debug":false,"base":"https:\/\/moodle.upm.es\/titulaciones\/oficiales\/lib\/yuilib\/3.17.2\/","comboBase":"https:\/\/moodle.upm.es\/titulaciones\/oficiales\/theme\/yui_combo.php?","combine":true,"filter":null,"insertBefore":"firstthemesheet","groups":{"yui2":{"base":"https:\/\/moodle.upm.es\/titulaciones\/oficiales\/lib\/yuilib\/2in3\/2.9.0\/build\/","comboBase":"https:\/\/moodle.upm.es\/titulaciones\/oficiales\/theme\/yui_combo.php?","combine":true,"ext":false,"root":"2in3\/2.9.0\/build\/","patterns":{"yui2-":{"group":"yui2","configFn":yui1ConfigFn}}},"moodle":{"name":"moodle","base":"https:\/\/moodle.upm.es\/titulaciones\/oficiales\/theme\/yui_combo.php?m\/1623142191\/","combine":true,"comboBase":"https:\/\/moodle.upm.es\/titulaciones\/oficiales\/theme\/yui_combo.php?","ext":false,"root":"m\/1623142191\/","patterns":{"moodle-":{"group":"moodle","configFn":yui2ConfigFn}},"filter":null,"modules":{"moodle-core-blocks":{"requires":["base","node","io","dom","dd","dd-scroll","moodle-core-dragdrop","moodle-core-notification"]},"moodle-core-chooserdialogue":{"requires":["base","panel","moodle-core-notification"]},"moodle-core-formchangechecker":{"requires":["base","event-focus","moodle-core-event"]},"moodle-core-tooltip":{"requires":["base","node","io-base","moodle-core-notification-dialogue","json-parse","widget-position","widget-position-align","event-outside","cache-base"]},"moodle-core-dragdrop":{"requires":["base","node","io","dom","dd","event-key","event-focus","moodle-core-notification"]},"moodle-core-lockscroll":{"requires":["plugin","base-build"]},"moodle-core-maintenancemodetimer":{"requires":["base","node"]},"moodle-core-popuphelp":{"requires":["moodle-core-tooltip"]},"moodle-core-languninstallconfirm":{"requires":["base","node","moodle-core-notification-confirm","moodle-core-notification-alert"]},"moodle-core-event":{"requires":["event-custom"]},"moodle-core-handlebars":{"condition":{"trigger":"handlebars","when":"after"}},"moodle-core-notification":{"requires":["moodle-core-notification-dialogue","moodle-core-notification-alert","moodle-core-notification-confirm","moodle-core-notification-exception","moodle-core-notification-ajaxexception"]},"moodle-core-notification-dialogue":{"requires":["base","node","panel","escape","event-key","dd-plugin","moodle-core-widget-focusafterclose","moodle-core-lockscroll"]},"moodle-core-notification-alert":{"requires":["moodle-core-notification-dialogue"]},"moodle-core-notification-confirm":{"requires":["moodle-core-notification-dialogue"]},"moodle-core-notification-exception":{"requires":["moodle-core-notification-dialogue"]},"moodle-core-notification-ajaxexception":{"requires":["moodle-core-notification-dialogue"]},"moodle-core-actionmenu":{"requires":["base","event","node-event-simulate"]},"moodle-core_availability-form":{"requires":["base","node","event","event-delegate","panel","moodle-core-notification-dialogue","json"]},"moodle-backup-confirmcancel":{"requires":["node","node-event-simulate","moodle-core-notification-confirm"]},"moodle-backup-backupselectall":{"requires":["node","event","node-event-simulate","anim"]},"moodle-course-categoryexpander":{"requires":["node","event-key"]},"moodle-course-formatchooser":{"requires":["base","node","node-event-simulate"]},"moodle-course-dragdrop":{"requires":["base","node","io","dom","dd","dd-scroll","moodle-core-dragdrop","moodle-core-notification","moodle-course-coursebase","moodle-course-util"]},"moodle-course-management":{"requires":["base","node","io-base","moodle-core-notification-exception","json-parse","dd-constrain","dd-proxy","dd-drop","dd-delegate","node-event-delegate"]},"moodle-course-util":{"requires":["node"],"use":["moodle-course-util-base"],"submodules":{"moodle-course-util-base":{},"moodle-course-util-section":{"requires":["node","moodle-course-util-base"]},"moodle-course-util-cm":{"requires":["node","moodle-course-util-base"]}}},"moodle-form-passwordunmask":{"requires":[]},"moodle-form-dateselector":{"requires":["base","node","overlay","calendar"]},"moodle-form-shortforms":{"requires":["node","base","selector-css3","moodle-core-event"]},"moodle-question-preview":{"requires":["base","dom","event-delegate","event-key","core_question_engine"]},"moodle-question-searchform":{"requires":["base","node"]},"moodle-question-chooser":{"requires":["moodle-core-chooserdialogue"]},"moodle-availability_completion-form":{"requires":["base","node","event","moodle-core_availability-form"]},"moodle-availability_date-form":{"requires":["base","node","event","io","moodle-core_availability-form"]},"moodle-availability_grade-form":{"requires":["base","node","event","moodle-core_availability-form"]},"moodle-availability_group-form":{"requires":["base","node","event","moodle-core_availability-form"]},"moodle-availability_grouping-form":{"requires":["base","node","event","moodle-core_availability-form"]},"moodle-availability_profile-form":{"requires":["base","node","event","moodle-core_availability-form"]},"moodle-mod_assign-history":{"requires":["node","transition"]},"moodle-mod_attendance-groupfilter":{"requires":["base","node"]},"moodle-mod_offlinequiz-questionchooser":{"requires":["moodle-core-chooserdialogue","moodle-mod_offlinequiz-util","querystring-parse"]},"moodle-mod_offlinequiz-modform":{"requires":["base","node","event"]},"moodle-mod_offlinequiz-toolboxes":{"requires":["base","node","event","event-key","io","moodle-mod_offlinequiz-offlinequizbase","moodle-mod_offlinequiz-util-slot","moodle-core-notification-ajaxexception"]},"moodle-mod_offlinequiz-autosave":{"requires":["base","node","event","event-valuechange","node-event-delegate","io-form"]},"moodle-mod_offlinequiz-dragdrop":{"requires":["base","node","io","dom","dd","dd-scroll","moodle-core-dragdrop","moodle-core-notification","moodle-mod_offlinequiz-offlinequizbase","moodle-mod_offlinequiz-util-base","moodle-mod_offlinequiz-util-page","moodle-mod_offlinequiz-util-slot","moodle-course-util"]},"moodle-mod_offlinequiz-repaginate":{"requires":["base","event","node","io","moodle-core-notification-dialogue"]},"moodle-mod_offlinequiz-util":{"requires":["node"],"use":["moodle-mod_offlinequiz-util-base"],"submodules":{"moodle-mod_offlinequiz-util-base":{},"moodle-mod_offlinequiz-util-slot":{"requires":["node","moodle-mod_offlinequiz-util-base"]},"moodle-mod_offlinequiz-util-page":{"requires":["node","moodle-mod_offlinequiz-util-base"]}}},"moodle-mod_offlinequiz-offlinequizquestionbank":{"requires":["base","event","node","io","io-form","yui-later","moodle-question-qbankmanager","moodle-question-chooser","moodle-question-searchform","moodle-core-notification"]},"moodle-mod_offlinequiz-randomquestion":{"requires":["base","event","node","io","moodle-core-notification-dialogue"]},"moodle-mod_offlinequiz-offlinequizbase":{"requires":["base","node"]},"moodle-mod_quiz-questionchooser":{"requires":["moodle-core-chooserdialogue","moodle-mod_quiz-util","querystring-parse"]},"moodle-mod_quiz-modform":{"requires":["base","node","event"]},"moodle-mod_quiz-toolboxes":{"requires":["base","node","event","event-key","io","moodle-mod_quiz-quizbase","moodle-mod_quiz-util-slot","moodle-core-notification-ajaxexception"]},"moodle-mod_quiz-autosave":{"requires":["base","node","event","event-valuechange","node-event-delegate","io-form"]},"moodle-mod_quiz-dragdrop":{"requires":["base","node","io","dom","dd","dd-scroll","moodle-core-dragdrop","moodle-core-notification","moodle-mod_quiz-quizbase","moodle-mod_quiz-util-base","moodle-mod_quiz-util-page","moodle-mod_quiz-util-slot","moodle-course-util"]},"moodle-mod_quiz-quizbase":{"requires":["base","node"]},"moodle-mod_quiz-util":{"requires":["node","moodle-core-actionmenu"],"use":["moodle-mod_quiz-util-base"],"submodules":{"moodle-mod_quiz-util-base":{},"moodle-mod_quiz-util-slot":{"requires":["node","moodle-mod_quiz-util-base"]},"moodle-mod_quiz-util-page":{"requires":["node","moodle-mod_quiz-util-base"]}}},"moodle-message_airnotifier-toolboxes":{"requires":["base","node","io"]},"moodle-filter_glossary-autolinker":{"requires":["base","node","io-base","json-parse","event-delegate","overlay","moodle-core-event","moodle-core-notification-alert","moodle-core-notification-exception","moodle-core-notification-ajaxexception"]},"moodle-filter_mathjaxloader-loader":{"requires":["moodle-core-event"]},"moodle-editor_atto-editor":{"requires":["node","transition","io","overlay","escape","event","event-simulate","event-custom","node-event-html5","node-event-simulate","yui-throttle","moodle-core-notification-dialogue","moodle-core-notification-confirm","moodle-editor_atto-rangy","handlebars","timers","querystring-stringify"]},"moodle-editor_atto-plugin":{"requires":["node","base","escape","event","event-outside","handlebars","event-custom","timers","moodle-editor_atto-menu"]},"moodle-editor_atto-menu":{"requires":["moodle-core-notification-dialogue","node","event","event-custom"]},"moodle-editor_atto-rangy":{"requires":[]},"moodle-report_eventlist-eventfilter":{"requires":["base","event","node","node-event-delegate","datatable","autocomplete","autocomplete-filters"]},"moodle-report_loglive-fetchlogs":{"requires":["base","event","node","io","node-event-delegate"]},"moodle-gradereport_grader-gradereporttable":{"requires":["base","node","event","handlebars","overlay","event-hover"]},"moodle-gradereport_history-userselector":{"requires":["escape","event-delegate","event-key","handlebars","io-base","json-parse","moodle-core-notification-dialogue"]},"moodle-tool_capability-search":{"requires":["base","node"]},"moodle-tool_lp-dragdrop-reorder":{"requires":["moodle-core-dragdrop"]},"moodle-tool_monitor-dropdown":{"requires":["base","event","node"]},"moodle-assignfeedback_editpdf-editor":{"requires":["base","event","node","io","graphics","json","event-move","event-resize","transition","querystring-stringify-simple","moodle-core-notification-dialog","moodle-core-notification-alert","moodle-core-notification-warning","moodle-core-notification-exception","moodle-core-notification-ajaxexception"]},"moodle-atto_accessibilitychecker-button":{"requires":["color-base","moodle-editor_atto-plugin"]},"moodle-atto_accessibilityhelper-button":{"requires":["moodle-editor_atto-plugin"]},"moodle-atto_align-button":{"requires":["moodle-editor_atto-plugin"]},"moodle-atto_bold-button":{"requires":["moodle-editor_atto-plugin"]},"moodle-atto_charmap-button":{"requires":["moodle-editor_atto-plugin"]},"moodle-atto_clear-button":{"requires":["moodle-editor_atto-plugin"]},"moodle-atto_collapse-button":{"requires":["moodle-editor_atto-plugin"]},"moodle-atto_emojipicker-button":{"requires":["moodle-editor_atto-plugin"]},"moodle-atto_emoticon-button":{"requires":["moodle-editor_atto-plugin"]},"moodle-atto_equation-button":{"requires":["moodle-editor_atto-plugin","moodle-core-event","io","event-valuechange","tabview","array-extras"]},"moodle-atto_fullscreen-button":{"requires":["event-resize","moodle-editor_atto-plugin"]},"moodle-atto_h5p-button":{"requires":["moodle-editor_atto-plugin"]},"moodle-atto_html-beautify":{},"moodle-atto_html-button":{"requires":["promise","moodle-editor_atto-plugin","moodle-atto_html-beautify","moodle-atto_html-codemirror","event-valuechange"]},"moodle-atto_html-codemirror":{"requires":["moodle-atto_html-codemirror-skin"]},"moodle-atto_image-button":{"requires":["moodle-editor_atto-plugin"]},"moodle-atto_indent-button":{"requires":["moodle-editor_atto-plugin"]},"moodle-atto_italic-button":{"requires":["moodle-editor_atto-plugin"]},"moodle-atto_link-button":{"requires":["moodle-editor_atto-plugin"]},"moodle-atto_managefiles-usedfiles":{"requires":["node","escape"]},"moodle-atto_managefiles-button":{"requires":["moodle-editor_atto-plugin"]},"moodle-atto_matrix-button":{"requires":["moodle-editor_atto-plugin","moodle-core-event","io","event-valuechange","tabview","array-extras"]},"moodle-atto_media-button":{"requires":["moodle-editor_atto-plugin","moodle-form-shortforms"]},"moodle-atto_noautolink-button":{"requires":["moodle-editor_atto-plugin"]},"moodle-atto_orderedlist-button":{"requires":["moodle-editor_atto-plugin"]},"moodle-atto_recordrtc-button":{"requires":["moodle-editor_atto-plugin","moodle-atto_recordrtc-recording"]},"moodle-atto_recordrtc-recording":{"requires":["moodle-atto_recordrtc-button"]},"moodle-atto_rtl-button":{"requires":["moodle-editor_atto-plugin"]},"moodle-atto_strike-button":{"requires":["moodle-editor_atto-plugin"]},"moodle-atto_subscript-button":{"requires":["moodle-editor_atto-plugin"]},"moodle-atto_superscript-button":{"requires":["moodle-editor_atto-plugin"]},"moodle-atto_table-button":{"requires":["moodle-editor_atto-plugin","moodle-editor_atto-menu","event","event-valuechange"]},"moodle-atto_title-button":{"requires":["moodle-editor_atto-plugin"]},"moodle-atto_underline-button":{"requires":["moodle-editor_atto-plugin"]},"moodle-atto_undo-button":{"requires":["moodle-editor_atto-plugin"]},"moodle-atto_unorderedlist-button":{"requires":["moodle-editor_atto-plugin"]},"moodle-atto_wordimport-button":{"requires":["moodle-editor_atto-plugin"]}}},"gallery":{"name":"gallery","base":"https:\/\/moodle.upm.es\/titulaciones\/oficiales\/lib\/yuilib\/gallery\/","combine":true,"comboBase":"https:\/\/moodle.upm.es\/titulaciones\/oficiales\/theme\/yui_combo.php?","ext":false,"root":"gallery\/1623142191\/","patterns":{"gallery-":{"group":"gallery"}}}},"modules":{"core_filepicker":{"name":"core_filepicker","fullpath":"https:\/\/moodle.upm.es\/titulaciones\/oficiales\/lib\/javascript.php\/1623142191\/repository\/filepicker.js","requires":["base","node","node-event-simulate","json","async-queue","io-base","io-upload-iframe","io-form","yui2-treeview","panel","cookie","datatable","datatable-sort","resize-plugin","dd-plugin","escape","moodle-core_filepicker","moodle-core-notification-dialogue"]},"core_comment":{"name":"core_comment","fullpath":"https:\/\/moodle.upm.es\/titulaciones\/oficiales\/lib\/javascript.php\/1623142191\/comment\/comment.js","requires":["base","io-base","node","json","yui2-animation","overlay","escape"]},"mathjax":{"name":"mathjax","fullpath":"https:\/\/cdn.jsdelivr.net\/npm\/mathjax@2.7.8\/MathJax.js?delayStartupUntil=configured"}}};
M.yui.loader = {modules: {}};

//]]>
</script>

<meta name="moodle-validation" content="011bc55b9caa41eded0581338c58da4a">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
</head>
<body  id="page-mod-resource-view" class="format-topics  path-mod path-mod-resource gecko dir-ltr lang-es yui-skin-sam yui3-skin-sam moodle-upm-es--titulaciones-oficiales pagelayout-incourse course-5810 context-8143720 cmid-1141685 category-457 drawer-open-left">
<div class="toast-wrapper mx-auto py-0 fixed-top" role="status" aria-live="polite"></div>

<div id="page-wrapper" class="d-print-block">

    <div>
    <a class="sr-only sr-only-focusable" href="#maincontent">Salta al contenido principal</a>
</div><script src="https://moodle.upm.es/titulaciones/oficiales/lib/javascript.php/1623142191/lib/babel-polyfill/polyfill.min.js"></script>
<script src="https://moodle.upm.es/titulaciones/oficiales/lib/javascript.php/1623142191/lib/polyfills/polyfill.js"></script>
<script src="https://moodle.upm.es/titulaciones/oficiales/theme/yui_combo.php?rollup/3.17.2/yui-moodlesimple-min.js"></script><script src="https://moodle.upm.es/titulaciones/oficiales/lib/javascript.php/1623142191/lib/javascript-static.js"></script>
<script>
//<![CDATA[
document.body.className += ' jsenabled';
//]]>
</script>



    <nav class="fixed-top navbar navbar-light bg-white navbar-expand moodle-has-zindex" aria-label="Navegación del sitio">
    
            <div data-region="drawer-toggle" class="d-inline-block mr-3">
                <button aria-expanded="true" aria-controls="nav-drawer" type="button" class="btn nav-link float-sm-left mr-1 btn-light bg-gray" data-action="toggle-drawer" data-side="left" data-preference="drawer-open-nav"><i class="icon fa fa-bars fa-fw " aria-hidden="true"  ></i><span class="sr-only">Panel lateral</span></button>
            </div>
    
            <a href="https://moodle.upm.es/titulaciones/oficiales" class="navbar-brand aabtn 
                    d-none d-sm-inline
                    ">
                <span class="site-name d-none d-md-inline">UPM ESTUDIOS OFICIALES</span>
            </a>
    
            <ul class="navbar-nav d-none d-md-flex">
                <!-- custom_menu -->
                <li class="dropdown nav-item">
    <a class="dropdown-toggle nav-link" id="drop-down-60bf3510cd34f60bf3510cb0c54" data-toggle="dropdown" aria-haspopup="true" aria-expanded="false" href="#"  aria-controls="drop-down-menu-60bf3510cd34f60bf3510cb0c54">
        UPM
    </a>
    <div class="dropdown-menu" role="menu" id="drop-down-menu-60bf3510cd34f60bf3510cb0c54" aria-labelledby="drop-down-60bf3510cd34f60bf3510cb0c54">
                <a class="dropdown-item" role="menuitem" href="https://www.upm.es "target="_blank" >Web de la UPM</a>
                <a class="dropdown-item" role="menuitem" href="https://www.upm.es/politecnica_virtual/ "target="_blank" >Politécnica Virtual</a>
                <a class="dropdown-item" role="menuitem" href="https://ingenio.upm.es "target="_blank" >Biblioteca UPM</a>
                <a class="dropdown-item" role="menuitem" href="https://www.upm.es/webmail_personal/ "target="_blank" >Correo electrónico PAS y PDI</a>
                <a class="dropdown-item" role="menuitem" href="https://www.upm.es/webmail_alumnos/ "target="_blank" >Correo electrónico Estudiantes</a>
                <a class="dropdown-item" role="menuitem" href="https://moodle.upm.es/titulaciones/propias/" >Moodle - Titulaciones propias</a>
                <a class="dropdown-item" role="menuitem" href="https://moodle.upm.es/puestaapunto/" >Moodle - Puesta a punto</a>
                <a class="dropdown-item" role="menuitem" href="https://moodle.upm.es/formacion/" >Moodle - Formación UPM</a>
    </div>
</li><li class="dropdown nav-item">
    <a class="dropdown-toggle nav-link" id="drop-down-60bf3510cd3d660bf3510cb0c55" data-toggle="dropdown" aria-haspopup="true" aria-expanded="false" href="#"  aria-controls="drop-down-menu-60bf3510cd3d660bf3510cb0c55">
        Ayuda
    </a>
    <div class="dropdown-menu" role="menu" id="drop-down-menu-60bf3510cd3d660bf3510cb0c55" aria-labelledby="drop-down-60bf3510cd3d660bf3510cb0c55">
                <a class="dropdown-item" role="menuitem" href="https://moodle.upm.es/titulaciones/oficiales/mod/page/view.php?id=867182" >Contacto y preguntas frecuentes</a>
                <a class="dropdown-item" role="menuitem" href="https://moodle.upm.es/titulaciones/oficiales/course/view.php?id=242" >Ayuda y documentación para profesores</a>
                <a class="dropdown-item" role="menuitem" href="https://moodle.upm.es/titulaciones/oficiales/course/view.php?id=3048" >Ayuda y documentación para estudiantes</a>
                <a class="dropdown-item" role="menuitem" href="https://moodle.upm.es/titulaciones/oficiales/course/view.php?id=9106" >Curso básico online para el profesor</a>
                <a class="dropdown-item" role="menuitem" href="https://docs.moodle.org/all/es/P%C3%A1gina_Principal" >Documentación oficial de Moodle</a>
                <a class="dropdown-item" role="menuitem" href="http://oa.upm.es/65760/7/Manual_Moodle_3_9.pdf" >Manual para el profesor</a>
    </div>
</li><li class="dropdown nav-item">
    <a class="dropdown-toggle nav-link" id="drop-down-60bf3510cd43960bf3510cb0c56" data-toggle="dropdown" aria-haspopup="true" aria-expanded="false" href="#" title="Idioma" aria-controls="drop-down-menu-60bf3510cd43960bf3510cb0c56">
        Español - Internacional ‎(es)‎
    </a>
    <div class="dropdown-menu" role="menu" id="drop-down-menu-60bf3510cd43960bf3510cb0c56" aria-labelledby="drop-down-60bf3510cd43960bf3510cb0c56">
                <a class="dropdown-item" role="menuitem" href="https://moodle.upm.es/titulaciones/oficiales/mod/resource/view.php?id=1141685&amp;lang=de" title="Deutsch ‎(de)‎">Deutsch ‎(de)‎</a>
                <a class="dropdown-item" role="menuitem" href="https://moodle.upm.es/titulaciones/oficiales/mod/resource/view.php?id=1141685&amp;lang=en" title="English ‎(en)‎">English ‎(en)‎</a>
                <a class="dropdown-item" role="menuitem" href="https://moodle.upm.es/titulaciones/oficiales/mod/resource/view.php?id=1141685&amp;lang=es" title="Español - Internacional ‎(es)‎">Español - Internacional ‎(es)‎</a>
                <a class="dropdown-item" role="menuitem" href="https://moodle.upm.es/titulaciones/oficiales/mod/resource/view.php?id=1141685&amp;lang=fr" title="Français ‎(fr)‎">Français ‎(fr)‎</a>
                <a class="dropdown-item" role="menuitem" href="https://moodle.upm.es/titulaciones/oficiales/mod/resource/view.php?id=1141685&amp;lang=it" title="Italiano ‎(it)‎">Italiano ‎(it)‎</a>
                <a class="dropdown-item" role="menuitem" href="https://moodle.upm.es/titulaciones/oficiales/mod/resource/view.php?id=1141685&amp;lang=pt" title="Português - Portugal ‎(pt)‎">Português - Portugal ‎(pt)‎</a>
                <a class="dropdown-item" role="menuitem" href="https://moodle.upm.es/titulaciones/oficiales/mod/resource/view.php?id=1141685&amp;lang=zh_cn" title="简体中文 ‎(zh_cn)‎">简体中文 ‎(zh_cn)‎</a>
    </div>
</li>
                <!-- page_heading_menu -->
                
            </ul>
            <ul class="nav navbar-nav ml-auto">
                <li class="d-none d-lg-block">
                    
                </li>
                <!-- navbar_plugin_output -->
                <li class="nav-item">
                    <div class="popover-region collapsed popover-region-notifications"
    id="nav-notification-popover-container" data-userid="18361061"
    data-region="popover-region">
    <div class="popover-region-toggle nav-link"
        data-region="popover-region-toggle"
        role="button"
        aria-controls="popover-region-container-60bf3510ce15a60bf3510cb0c57"
        aria-haspopup="true"
        aria-label="Mostrar la ventana de notificaciones cuando no hay ninguna"
        tabindex="0">
                <i class="icon fa fa-bell fa-fw "  title="Mostrar/ocultar menú de notificaciones" aria-label="Mostrar/ocultar menú de notificaciones"></i>
        <div class="count-container hidden" data-region="count-container"
        aria-label="There are 0 unread notifications">0</div>

    </div>
    <div 
        id="popover-region-container-60bf3510ce15a60bf3510cb0c57"
        class="popover-region-container"
        data-region="popover-region-container"
        aria-expanded="false"
        aria-hidden="true"
        aria-label="Ventana de notificación"
        role="region">
        <div class="popover-region-header-container">
            <h3 class="popover-region-header-text" data-region="popover-region-header-text">Notificaciones</h3>
            <div class="popover-region-header-actions" data-region="popover-region-header-actions">        <a class="mark-all-read-button"
           href="#"
           title="Marcar como leído"
           data-action="mark-all-read"
           role="button"
           aria-label="Marcar como leído">
            <span class="normal-icon"><i class="icon fa fa-check fa-fw " aria-hidden="true"  ></i></span>
            <span class="loading-icon icon-no-margin"><i class="icon fa fa-circle-o-notch fa-spin fa-fw "  title="Enviando" aria-label="Enviando"></i></span>
        </a>
        <a href="https://moodle.upm.es/titulaciones/oficiales/message/notificationpreferences.php?userid=18361061"
           title="Preferencias de notificación"
           aria-label="Preferencias de notificación">
            <i class="icon fa fa-cog fa-fw " aria-hidden="true"  ></i>
        </a>
</div>
        </div>
        <div class="popover-region-content-container" data-region="popover-region-content-container">
            <div class="popover-region-content" data-region="popover-region-content">
                        <div class="all-notifications"
            data-region="all-notifications"
            role="log"
            aria-busy="false"
            aria-atomic="false"
            aria-relevant="additions"></div>
        <div class="empty-message" tabindex="0" data-region="empty-message">No tienes notificaciones</div>

            </div>
            <span class="loading-icon icon-no-margin"><i class="icon fa fa-circle-o-notch fa-spin fa-fw "  title="Enviando" aria-label="Enviando"></i></span>
        </div>
                <a class="see-all-link"
                    href="https://moodle.upm.es/titulaciones/oficiales/message/output/popup/notifications.php">
                    <div class="popover-region-footer-container">
                        <div class="popover-region-seeall-text">Ver todo</div>
                    </div>
                </a>
    </div>
</div><div class="popover-region collapsed" data-region="popover-region-messages">
    <a id="message-drawer-toggle-60bf3510ced9060bf3510cb0c58" class="nav-link d-inline-block popover-region-toggle position-relative" href="#"
            role="button">
        <i class="icon fa fa-comment fa-fw "  title="Mostrar/ocultar menú de mensajes" aria-label="Mostrar/ocultar menú de mensajes"></i>
        <div class="count-container hidden" data-region="count-container"
        aria-label="There are 0 unread conversations">0</div>
    </a>
    <span class="sr-only sr-only-focusable" data-region="jumpto" tabindex="-1"></span></div>
                </li>
                <!-- user_menu -->
                <li class="nav-item d-flex align-items-center">
                    <div class="usermenu"><div class="action-menu moodle-actionmenu nowrap-items d-inline" id="action-menu-1" data-enhance="moodle-core-actionmenu">

        <div class="menubar d-flex " id="action-menu-1-menubar" role="menubar">

            


                <div class="action-menu-trigger">
                    <div class="dropdown">
                        <a href="#" tabindex="0" class="d-inline-block  dropdown-toggle icon-no-margin" id="action-menu-toggle-1" aria-label="Menú de usuario" data-toggle="dropdown" role="button" aria-haspopup="true" aria-expanded="false" aria-controls="action-menu-1-menu">
                            
                            <span class="userbutton"><span class="usertext mr-1">MALISEK MIROSLAV</span><span class="avatars"><span class="avatar current"><img src="https://moodle.upm.es/titulaciones/oficiales/theme/image.php/boost/core/1623142191/u/f2" class="userpicture defaultuserpic" width="35" height="35" alt="" /></span></span></span>
                                
                            <b class="caret"></b>
                        </a>
                            <div class="dropdown-menu dropdown-menu-right menu  align-tr-br" id="action-menu-1-menu" data-rel="menu-content" aria-labelledby="action-menu-toggle-1" role="menu" data-align="tr-br">
                                                                <a href="https://moodle.upm.es/titulaciones/oficiales/my/" class="dropdown-item menu-action" role="menuitem" data-title="mymoodle,admin" aria-labelledby="actionmenuaction-1">
                                <i class="icon fa fa-tachometer fa-fw " aria-hidden="true"  ></i>
                                <span class="menu-action-text" id="actionmenuaction-1">Área personal</span>
                        </a>
                    <div class="dropdown-divider" role="presentation"><span class="filler">&nbsp;</span></div>
                                                                <a href="https://moodle.upm.es/titulaciones/oficiales/user/profile.php?id=18361061" class="dropdown-item menu-action" role="menuitem" data-title="profile,moodle" aria-labelledby="actionmenuaction-2">
                                <i class="icon fa fa-user fa-fw " aria-hidden="true"  ></i>
                                <span class="menu-action-text" id="actionmenuaction-2">Perfil</span>
                        </a>
                                                                <a href="https://moodle.upm.es/titulaciones/oficiales/grade/report/overview/index.php" class="dropdown-item menu-action" role="menuitem" data-title="grades,grades" aria-labelledby="actionmenuaction-3">
                                <i class="icon fa fa-table fa-fw " aria-hidden="true"  ></i>
                                <span class="menu-action-text" id="actionmenuaction-3">Calificaciones</span>
                        </a>
                                                                <a href="https://moodle.upm.es/titulaciones/oficiales/message/index.php" class="dropdown-item menu-action" role="menuitem" data-title="messages,message" aria-labelledby="actionmenuaction-4">
                                <i class="icon fa fa-comment fa-fw " aria-hidden="true"  ></i>
                                <span class="menu-action-text" id="actionmenuaction-4">Mensajes</span>
                        </a>
                                                                <a href="https://moodle.upm.es/titulaciones/oficiales/user/preferences.php" class="dropdown-item menu-action" role="menuitem" data-title="preferences,moodle" aria-labelledby="actionmenuaction-5">
                                <i class="icon fa fa-wrench fa-fw " aria-hidden="true"  ></i>
                                <span class="menu-action-text" id="actionmenuaction-5">Preferencias</span>
                        </a>
                    <div class="dropdown-divider" role="presentation"><span class="filler">&nbsp;</span></div>
                                                                <a href="https://moodle.upm.es/titulaciones/oficiales/login/logout.php?sesskey=FjPb0IgJ50" class="dropdown-item menu-action" role="menuitem" data-title="logout,moodle" aria-labelledby="actionmenuaction-6">
                                <i class="icon fa fa-sign-out fa-fw " aria-hidden="true"  ></i>
                                <span class="menu-action-text" id="actionmenuaction-6">Cerrar sesión</span>
                        </a>
                            </div>
                    </div>
                </div>

        </div>

</div></div>
                </li>
            </ul>
            <!-- search_box -->
    </nav>
    
    <div id="nav-drawer" data-region="drawer" class="d-print-none moodle-has-zindex " aria-hidden="false" tabindex="-1">
        <nav class="list-group" aria-label="RobotPercComp-GII">
            <ul>
                    <li>
                        <a class="list-group-item list-group-item-action  " href="https://moodle.upm.es/titulaciones/oficiales/course/view.php?id=5810" data-key="coursehome" data-isexpandable="0" data-indent="0" data-showdivider="0" data-type="60" data-nodetype="0" data-collapse="0" data-forceopen="0" data-isactive="0" data-hidden="0" data-preceedwithhr="0" >
                            <div class="ml-0">
                                <div class="media">
                                        <span class="media-left">
                                            <i class="icon fa fa-graduation-cap fa-fw " aria-hidden="true"  ></i>
                                        </span>
                                    <span class="media-body ">RobotPercComp-GII</span>
                                </div>
                            </div>
                        </a>
                    </li>
                    <li>
                        <a class="list-group-item list-group-item-action  " href="https://moodle.upm.es/titulaciones/oficiales/user/index.php?id=5810" data-key="participants" data-isexpandable="0" data-indent="0" data-showdivider="0" data-type="90" data-nodetype="1" data-collapse="0" data-forceopen="0" data-isactive="0" data-hidden="0" data-preceedwithhr="0" data-parent-key="5810">
                            <div class="ml-0">
                                <div class="media">
                                        <span class="media-left">
                                            <i class="icon fa fa-users fa-fw " aria-hidden="true"  ></i>
                                        </span>
                                    <span class="media-body ">Participantes</span>
                                </div>
                            </div>
                        </a>
                    </li>
                    <li>
                        <a class="list-group-item list-group-item-action  " href="https://moodle.upm.es/titulaciones/oficiales/badges/view.php?type=2&amp;id=5810" data-key="badgesview" data-isexpandable="0" data-indent="0" data-showdivider="0" data-type="70" data-nodetype="0" data-collapse="0" data-forceopen="0" data-isactive="0" data-hidden="0" data-preceedwithhr="0" data-parent-key="5810">
                            <div class="ml-0">
                                <div class="media">
                                        <span class="media-left">
                                            <i class="icon fa fa-shield fa-fw " aria-hidden="true"  ></i>
                                        </span>
                                    <span class="media-body ">Insignias</span>
                                </div>
                            </div>
                        </a>
                    </li>
                    <li>
                        <a class="list-group-item list-group-item-action  " href="https://moodle.upm.es/titulaciones/oficiales/grade/report/index.php?id=5810" data-key="grades" data-isexpandable="0" data-indent="0" data-showdivider="0" data-type="70" data-nodetype="0" data-collapse="0" data-forceopen="0" data-isactive="0" data-hidden="0" data-preceedwithhr="0" data-parent-key="5810">
                            <div class="ml-0">
                                <div class="media">
                                        <span class="media-left">
                                            <i class="icon fa fa-table fa-fw " aria-hidden="true"  ></i>
                                        </span>
                                    <span class="media-body ">Calificaciones</span>
                                </div>
                            </div>
                        </a>
                    </li>
                    <li>
                        <a class="list-group-item list-group-item-action  " href="https://moodle.upm.es/titulaciones/oficiales/course/view.php?id=5810#section-0" data-key="78577" data-isexpandable="0" data-indent="0" data-showdivider="0" data-type="30" data-nodetype="1" data-collapse="0" data-forceopen="0" data-isactive="0" data-hidden="0" data-preceedwithhr="0" data-parent-key="5810">
                            <div class="ml-0">
                                <div class="media">
                                        <span class="media-left">
                                            <i class="icon fa fa-folder-o fa-fw " aria-hidden="true"  ></i>
                                        </span>
                                    <span class="media-body ">Robótica y Percepción GII  - Curso 2020/21</span>
                                </div>
                            </div>
                        </a>
                    </li>
                    <li>
                        <a class="list-group-item list-group-item-action  " href="https://moodle.upm.es/titulaciones/oficiales/course/view.php?id=5810#section-1" data-key="78578" data-isexpandable="0" data-indent="0" data-showdivider="0" data-type="30" data-nodetype="1" data-collapse="0" data-forceopen="0" data-isactive="0" data-hidden="0" data-preceedwithhr="0" data-parent-key="5810">
                            <div class="ml-0">
                                <div class="media">
                                        <span class="media-left">
                                            <i class="icon fa fa-folder-o fa-fw " aria-hidden="true"  ></i>
                                        </span>
                                    <span class="media-body ">Tema 1 Introducción (Nik Swoboda)</span>
                                </div>
                            </div>
                        </a>
                    </li>
                    <li>
                        <a class="list-group-item list-group-item-action  " href="https://moodle.upm.es/titulaciones/oficiales/course/view.php?id=5810#section-2" data-key="106400" data-isexpandable="0" data-indent="0" data-showdivider="0" data-type="30" data-nodetype="1" data-collapse="0" data-forceopen="0" data-isactive="0" data-hidden="0" data-preceedwithhr="0" data-parent-key="5810">
                            <div class="ml-0">
                                <div class="media">
                                        <span class="media-left">
                                            <i class="icon fa fa-folder-o fa-fw " aria-hidden="true"  ></i>
                                        </span>
                                    <span class="media-body ">Tema 2 Navegación (Nik Swoboda)</span>
                                </div>
                            </div>
                        </a>
                    </li>
                    <li>
                        <a class="list-group-item list-group-item-action  " href="https://moodle.upm.es/titulaciones/oficiales/course/view.php?id=5810#section-3" data-key="116086" data-isexpandable="0" data-indent="0" data-showdivider="0" data-type="30" data-nodetype="1" data-collapse="0" data-forceopen="0" data-isactive="0" data-hidden="0" data-preceedwithhr="0" data-parent-key="5810">
                            <div class="ml-0">
                                <div class="media">
                                        <span class="media-left">
                                            <i class="icon fa fa-folder-o fa-fw " aria-hidden="true"  ></i>
                                        </span>
                                    <span class="media-body ">Tema 3. Análisis de imagen (Luis Baumela)</span>
                                </div>
                            </div>
                        </a>
                    </li>
                    <li>
                        <a class="list-group-item list-group-item-action active " href="https://moodle.upm.es/titulaciones/oficiales/course/view.php?id=5810#section-5" data-key="78581" data-isexpandable="0" data-indent="0" data-showdivider="0" data-type="30" data-nodetype="1" data-collapse="0" data-forceopen="1" data-isactive="1" data-hidden="0" data-preceedwithhr="0" data-parent-key="5810">
                            <div class="ml-0">
                                <div class="media">
                                        <span class="media-left">
                                            <i class="icon fa fa-folder-o fa-fw " aria-hidden="true"  ></i>
                                        </span>
                                    <span class="media-body font-weight-bold">Tema 4 Integración</span>
                                </div>
                            </div>
                        </a>
                    </li>
                    </ul>
                    </nav>
                    <nav class="list-group mt-1" aria-label="Sitio">
                    <ul>
                    <li>
                        <a class="list-group-item list-group-item-action  " href="https://moodle.upm.es/titulaciones/oficiales/my/" data-key="myhome" data-isexpandable="0" data-indent="0" data-showdivider="1" data-type="1" data-nodetype="1" data-collapse="0" data-forceopen="1" data-isactive="0" data-hidden="0" data-preceedwithhr="0" >
                            <div class="ml-0">
                                <div class="media">
                                        <span class="media-left">
                                            <i class="icon fa fa-tachometer fa-fw " aria-hidden="true"  ></i>
                                        </span>
                                    <span class="media-body ">Área personal</span>
                                </div>
                            </div>
                        </a>
                    </li>
                    <li>
                        <a class="list-group-item list-group-item-action  " href="https://moodle.upm.es/titulaciones/oficiales/?redirect=0" data-key="home" data-isexpandable="0" data-indent="0" data-showdivider="0" data-type="70" data-nodetype="0" data-collapse="0" data-forceopen="0" data-isactive="0" data-hidden="0" data-preceedwithhr="0" data-parent-key="myhome">
                            <div class="ml-0">
                                <div class="media">
                                        <span class="media-left">
                                            <i class="icon fa fa-home fa-fw " aria-hidden="true"  ></i>
                                        </span>
                                    <span class="media-body ">Inicio del sitio</span>
                                </div>
                            </div>
                        </a>
                    </li>
                    <li>
                        <a class="list-group-item list-group-item-action  " href="https://moodle.upm.es/titulaciones/oficiales/calendar/view.php?view=month&amp;course=5810" data-key="calendar" data-isexpandable="0" data-indent="0" data-showdivider="0" data-type="60" data-nodetype="0" data-collapse="0" data-forceopen="0" data-isactive="0" data-hidden="0" data-preceedwithhr="0" data-parent-key="1">
                            <div class="ml-0">
                                <div class="media">
                                        <span class="media-left">
                                            <i class="icon fa fa-calendar fa-fw " aria-hidden="true"  ></i>
                                        </span>
                                    <span class="media-body ">Calendario</span>
                                </div>
                            </div>
                        </a>
                    </li>
                    <li>
                        <a class="list-group-item list-group-item-action  " href="https://moodle.upm.es/titulaciones/oficiales/user/files.php" data-key="privatefiles" data-isexpandable="0" data-indent="0" data-showdivider="0" data-type="70" data-nodetype="0" data-collapse="0" data-forceopen="0" data-isactive="0" data-hidden="0" data-preceedwithhr="0" data-parent-key="1">
                            <div class="ml-0">
                                <div class="media">
                                        <span class="media-left">
                                            <i class="icon fa fa-file-o fa-fw " aria-hidden="true"  ></i>
                                        </span>
                                    <span class="media-body ">Archivos privados</span>
                                </div>
                            </div>
                        </a>
                    </li>
                    <li>
                        <div class="list-group-item " data-key="mycourses" data-isexpandable="1" data-indent="0" data-showdivider="0" data-type="0" data-nodetype="1" data-collapse="0" data-forceopen="1" data-isactive="0" data-hidden="0" data-preceedwithhr="0" data-parent-key="myhome">
                            <div class="ml-0">
                                <div class="media">
                                        <span class="media-left">
                                            <i class="icon fa fa-graduation-cap fa-fw " aria-hidden="true"  ></i>
                                        </span>
                                    <span class="media-body">Mis cursos</span>
                                </div>
                            </div>
                        </div>
                    </li>
                    <li>
                        <a class="list-group-item list-group-item-action  " href="https://moodle.upm.es/titulaciones/oficiales/course/view.php?id=6670" data-key="6670" data-isexpandable="1" data-indent="1" data-showdivider="0" data-type="20" data-nodetype="1" data-collapse="0" data-forceopen="0" data-isactive="0" data-hidden="0" data-preceedwithhr="0" data-parent-key="mycourses">
                            <div class="ml-1">
                                <div class="media">
                                        <span class="media-left">
                                            <i class="icon fa fa-graduation-cap fa-fw " aria-hidden="true"  ></i>
                                        </span>
                                    <span class="media-body ">ALGORITMICA NUMERICA 3S1M-A</span>
                                </div>
                            </div>
                        </a>
                    </li>
                    <li>
                        <a class="list-group-item list-group-item-action  " href="https://moodle.upm.es/titulaciones/oficiales/course/view.php?id=10706" data-key="10706" data-isexpandable="1" data-indent="1" data-showdivider="0" data-type="20" data-nodetype="1" data-collapse="0" data-forceopen="0" data-isactive="0" data-hidden="0" data-preceedwithhr="0" data-parent-key="mycourses">
                            <div class="ml-1">
                                <div class="media">
                                        <span class="media-left">
                                            <i class="icon fa fa-graduation-cap fa-fw " aria-hidden="true"  ></i>
                                        </span>
                                    <span class="media-body ">Bases de datos (GMI)</span>
                                </div>
                            </div>
                        </a>
                    </li>
                    <li>
                        <a class="list-group-item list-group-item-action  " href="https://moodle.upm.es/titulaciones/oficiales/course/view.php?id=8760" data-key="8760" data-isexpandable="1" data-indent="1" data-showdivider="0" data-type="20" data-nodetype="1" data-collapse="0" data-forceopen="0" data-isactive="0" data-hidden="0" data-preceedwithhr="0" data-parent-key="mycourses">
                            <div class="ml-1">
                                <div class="media">
                                        <span class="media-left">
                                            <i class="icon fa fa-graduation-cap fa-fw " aria-hidden="true"  ></i>
                                        </span>
                                    <span class="media-body ">BBDD_GMI</span>
                                </div>
                            </div>
                        </a>
                    </li>
                    <li>
                        <a class="list-group-item list-group-item-action  " href="https://moodle.upm.es/titulaciones/oficiales/course/view.php?id=5515" data-key="5515" data-isexpandable="1" data-indent="1" data-showdivider="0" data-type="20" data-nodetype="1" data-collapse="0" data-forceopen="0" data-isactive="0" data-hidden="0" data-preceedwithhr="0" data-parent-key="mycourses">
                            <div class="ml-1">
                                <div class="media">
                                        <span class="media-left">
                                            <i class="icon fa fa-graduation-cap fa-fw " aria-hidden="true"  ></i>
                                        </span>
                                    <span class="media-body ">DATA_ANALYTICS</span>
                                </div>
                            </div>
                        </a>
                    </li>
                    <li>
                        <a class="list-group-item list-group-item-action  " href="https://moodle.upm.es/titulaciones/oficiales/course/view.php?id=7236" data-key="7236" data-isexpandable="1" data-indent="1" data-showdivider="0" data-type="20" data-nodetype="1" data-collapse="0" data-forceopen="0" data-isactive="0" data-hidden="0" data-preceedwithhr="0" data-parent-key="mycourses">
                            <div class="ml-1">
                                <div class="media">
                                        <span class="media-left">
                                            <i class="icon fa fa-graduation-cap fa-fw " aria-hidden="true"  ></i>
                                        </span>
                                    <span class="media-body ">DISEÑO DE APLICACIONES WEB</span>
                                </div>
                            </div>
                        </a>
                    </li>
                    <li>
                        <a class="list-group-item list-group-item-action  " href="https://moodle.upm.es/titulaciones/oficiales/course/view.php?id=9753" data-key="9753" data-isexpandable="1" data-indent="1" data-showdivider="0" data-type="20" data-nodetype="1" data-collapse="0" data-forceopen="0" data-isactive="0" data-hidden="0" data-preceedwithhr="0" data-parent-key="mycourses">
                            <div class="ml-1">
                                <div class="media">
                                        <span class="media-left">
                                            <i class="icon fa fa-graduation-cap fa-fw " aria-hidden="true"  ></i>
                                        </span>
                                    <span class="media-body ">Estructura de Computadores (GII-DGII+ADE)</span>
                                </div>
                            </div>
                        </a>
                    </li>
                    <li>
                        <a class="list-group-item list-group-item-action  " href="https://moodle.upm.es/titulaciones/oficiales/course/view.php?id=6588" data-key="6588" data-isexpandable="1" data-indent="1" data-showdivider="0" data-type="20" data-nodetype="1" data-collapse="0" data-forceopen="0" data-isactive="0" data-hidden="0" data-preceedwithhr="0" data-parent-key="mycourses">
                            <div class="ml-1">
                                <div class="media">
                                        <span class="media-left">
                                            <i class="icon fa fa-graduation-cap fa-fw " aria-hidden="true"  ></i>
                                        </span>
                                    <span class="media-body ">INFORMATICA INDUSTRIAL</span>
                                </div>
                            </div>
                        </a>
                    </li>
                    <li>
                        <a class="list-group-item list-group-item-action  " href="https://moodle.upm.es/titulaciones/oficiales/course/view.php?id=7116" data-key="7116" data-isexpandable="1" data-indent="1" data-showdivider="0" data-type="20" data-nodetype="1" data-collapse="0" data-forceopen="0" data-isactive="0" data-hidden="0" data-preceedwithhr="0" data-parent-key="mycourses">
                            <div class="ml-1">
                                <div class="media">
                                        <span class="media-left">
                                            <i class="icon fa fa-graduation-cap fa-fw " aria-hidden="true"  ></i>
                                        </span>
                                    <span class="media-body ">PDS</span>
                                </div>
                            </div>
                        </a>
                    </li>
                    <li>
                        <a class="list-group-item list-group-item-action  " href="https://moodle.upm.es/titulaciones/oficiales/course/view.php?id=6606" data-key="6606" data-isexpandable="1" data-indent="1" data-showdivider="0" data-type="20" data-nodetype="1" data-collapse="0" data-forceopen="0" data-isactive="0" data-hidden="0" data-preceedwithhr="0" data-parent-key="mycourses">
                            <div class="ml-1">
                                <div class="media">
                                        <span class="media-left">
                                            <i class="icon fa fa-graduation-cap fa-fw " aria-hidden="true"  ></i>
                                        </span>
                                    <span class="media-body ">PROGRAMACION I</span>
                                </div>
                            </div>
                        </a>
                    </li>
                    <li>
                        <a class="list-group-item list-group-item-action  " href="https://moodle.upm.es/titulaciones/oficiales/course/view.php?id=6668" data-key="6668" data-isexpandable="1" data-indent="1" data-showdivider="0" data-type="20" data-nodetype="1" data-collapse="0" data-forceopen="0" data-isactive="0" data-hidden="0" data-preceedwithhr="0" data-parent-key="mycourses">
                            <div class="ml-1">
                                <div class="media">
                                        <span class="media-left">
                                            <i class="icon fa fa-graduation-cap fa-fw " aria-hidden="true"  ></i>
                                        </span>
                                    <span class="media-body ">PROGRAMACION PARA SISTEMAS (Todos los grados)</span>
                                </div>
                            </div>
                        </a>
                    </li>
                    <li>
                        <a class="list-group-item list-group-item-action  " href="https://moodle.upm.es/titulaciones/oficiales/course/view.php?id=6780" data-key="6780" data-isexpandable="1" data-indent="1" data-showdivider="0" data-type="20" data-nodetype="1" data-collapse="0" data-forceopen="0" data-isactive="0" data-hidden="0" data-preceedwithhr="0" data-parent-key="mycourses">
                            <div class="ml-1">
                                <div class="media">
                                        <span class="media-left">
                                            <i class="icon fa fa-graduation-cap fa-fw " aria-hidden="true"  ></i>
                                        </span>
                                    <span class="media-body ">RECONOCIMIENTO DE FORMAS</span>
                                </div>
                            </div>
                        </a>
                    </li>
                    <li>
                        <a class="list-group-item list-group-item-action  " href="https://moodle.upm.es/titulaciones/oficiales/course/view.php?id=5810" data-key="5810" data-isexpandable="1" data-indent="1" data-showdivider="0" data-type="20" data-nodetype="1" data-collapse="0" data-forceopen="1" data-isactive="0" data-hidden="0" data-preceedwithhr="0" data-parent-key="mycourses">
                            <div class="ml-1">
                                <div class="media">
                                        <span class="media-left">
                                            <i class="icon fa fa-graduation-cap fa-fw " aria-hidden="true"  ></i>
                                        </span>
                                    <span class="media-body ">RobotPercComp-GII</span>
                                </div>
                            </div>
                        </a>
                    </li>
                    <li>
                        <a class="list-group-item list-group-item-action  " href="https://moodle.upm.es/titulaciones/oficiales/course/view.php?id=5851" data-key="5851" data-isexpandable="1" data-indent="1" data-showdivider="0" data-type="20" data-nodetype="1" data-collapse="0" data-forceopen="0" data-isactive="0" data-hidden="0" data-preceedwithhr="0" data-parent-key="mycourses">
                            <div class="ml-1">
                                <div class="media">
                                        <span class="media-left">
                                            <i class="icon fa fa-graduation-cap fa-fw " aria-hidden="true"  ></i>
                                        </span>
                                    <span class="media-body ">SOS</span>
                                </div>
                            </div>
                        </a>
                    </li>
            </ul>
        </nav>
    </div>

    <div id="page" class="container-fluid d-print-block">
        <header id="page-header" class="row">
    <div class="col-12 pt-3 pb-3">
        <div class="card ">
            <div class="card-body ">
                <div class="d-sm-flex align-items-center">
                    <div class="mr-auto">
                        <div class="page-context-header"><div class="page-header-headings"><h1>Robótica y percepción computacional</h1></div></div>
                    </div>

                    <div class="header-actions-container flex-shrink-0" data-region="header-actions-container">
                    </div>
                </div>
                <div class="d-flex flex-wrap">
                    <div id="page-navbar">
                        <nav aria-label="Barra de navegación">
    <ol class="breadcrumb">
                <li class="breadcrumb-item">
                    <a href="https://moodle.upm.es/titulaciones/oficiales/my/"  >Área personal</a>
                </li>
        
                <li class="breadcrumb-item">Mis cursos</li>
        
                <li class="breadcrumb-item">
                    <a href="https://moodle.upm.es/titulaciones/oficiales/course/view.php?id=5810"  title="Robótica y percepción computacional">RobotPercComp-GII</a>
                </li>
        
                <li class="breadcrumb-item">
                    <a href="https://moodle.upm.es/titulaciones/oficiales/course/view.php?id=5810#section-5"  >Tema 4 Integración</a>
                </li>
        
                <li class="breadcrumb-item">
                    <a href="https://moodle.upm.es/titulaciones/oficiales/mod/resource/view.php?id=1141685" aria-current="page" title="Archivo">Fichero .world del segundo recorrido</a>
                </li>
        </ol>
</nav>
                    </div>
                    <div class="ml-auto d-flex">
                        
                    </div>
                    <div id="course-header">
                        
                    </div>
                </div>
            </div>
        </div>
    </div>
</header>

        <div id="page-content" class="row pb-3 d-print-block">
            <div id="region-main-box" class="col-12">
                <section id="region-main" class="has-blocks mb-3" aria-label="Contenido">

                    <span class="notifications" id="user-notifications"></span>
                    <div role="main"><span id="maincontent"></span><h2>Fichero .world del segundo recorrido</h2><div id="resourceintro" class="box py-3 mod_introbox"><div class="no-overflow"><p>Segundo recorrido<br></p></div></div><div class="resourceworkaround">Haga clic en <a href="https://moodle.upm.es/titulaciones/oficiales/pluginfile.php/8143720/mod_resource/content/2/finalExam-202021-line2-v01.world" onclick="this.target='_blank'">finalExam-202021-line2-v01.world</a> para ver el archivo.</div></div>
                    <div class="mt-5 mb-1 activity-navigation container-fluid">
<div class="row">
    <div class="col-md-4">        <div class="float-left">
                <a href="https://moodle.upm.es/titulaciones/oficiales/mod/resource/view.php?id=1345092&forceview=1" id="prev-activity-link" class="btn btn-link"  title="Zip del mundo para la entrega final (v02)" >&#x25C4; Zip del mundo para la entrega final (v02)</a>

        </div>
</div>
    <div class="col-md-4">        <div class="mdl-align">
            <div class="urlselect">
    <form method="post" action="https://moodle.upm.es/titulaciones/oficiales/course/jumpto.php" class="form-inline" id="url_select_f60bf3510cb0c517">
        <input type="hidden" name="sesskey" value="FjPb0IgJ50">
            <label for="jump-to-activity" class="sr-only">
                Ir a...
            </label>
        <select  id="jump-to-activity" class="custom-select urlselect" name="jump"
                 >
                    <option value="" selected>Ir a...</option>
                    <option value="/mod/resource/view.php?id=532110&amp;forceview=1" >Presentación de la asignatura</option>
                    <option value="/mod/resource/view.php?id=946346&amp;forceview=1" >Presentación de la practica</option>
                    <option value="/mod/page/view.php?id=847459&amp;forceview=1" >Guía de Instalación de Pyrobot, ROS y Stage</option>
                    <option value="/mod/forum/view.php?id=775306&amp;forceview=1" >Foro de noticias</option>
                    <option value="/mod/forum/view.php?id=775307&amp;forceview=1" >Foro de las practicas de R&amp;P</option>
                    <option value="/mod/wiki/view.php?id=1344073&amp;forceview=1" >Wiki del Robot Casero</option>
                    <option value="/mod/url/view.php?id=479747&amp;forceview=1" >Tutorial de Python</option>
                    <option value="/mod/url/view.php?id=1157059&amp;forceview=1" >Libro de Python 3</option>
                    <option value="/mod/wiki/view.php?id=979065&amp;forceview=1" >Wiki de las demos del día 8 de junio a las 9:00</option>
                    <option value="/mod/lti/view.php?id=1324367&amp;forceview=1" >Acceso a clases y tutorias en linea</option>
                    <option value="/mod/resource/view.php?id=1060955&amp;forceview=1" >Código (v4.2) para la primera practica</option>
                    <option value="/mod/resource/view.php?id=1276780&amp;forceview=1" >Enunciado de la primera practica</option>
                    <option value="/mod/assign/view.php?id=749856&amp;forceview=1" >Entrega de Control</option>
                    <option value="/mod/assign/view.php?id=883111&amp;forceview=1" >Entrega de julio de Control</option>
                    <option value="/mod/url/view.php?id=479752&amp;forceview=1" >Documentación OpenCV</option>
                    <option value="/mod/url/view.php?id=960328&amp;forceview=1" >Enlace librería de aprendizeje automático SciKitLearn</option>
                    <option value="/mod/folder/view.php?id=756738&amp;forceview=1" >Lecturas</option>
                    <option value="/mod/resource/view.php?id=854397&amp;forceview=1" >Transparencias clase 23-03-2021. Introducción a la visión para robots</option>
                    <option value="/mod/resource/view.php?id=1330281&amp;forceview=1" >Transparencias clase 06-04-2021. Formación de imagen</option>
                    <option value="/mod/resource/view.php?id=1330282&amp;forceview=1" >Transparencias clase 06-04-2021. Tareas python.</option>
                    <option value="/mod/resource/view.php?id=1330396&amp;forceview=1" >Codigo practica Python</option>
                    <option value="/mod/resource/view.php?id=963076&amp;forceview=1" >Tareas 6 al 12 de abril de 2021</option>
                    <option value="/mod/assign/view.php?id=1330704&amp;forceview=1" >Entrega de resultados tareas semana 6 al 12 de abril</option>
                    <option value="/mod/resource/view.php?id=857613&amp;forceview=1" >Transparencias clase 13-04-2021. Segmentación de imágenes y clasificación</option>
                    <option value="/mod/resource/view.php?id=859312&amp;forceview=1" >Tareas 13 al 19 de abril</option>
                    <option value="/mod/assign/view.php?id=1334944&amp;forceview=1" >Entrega resultados tareas 12 al 19 de abril (19-04-2021 23:55)</option>
                    <option value="/mod/resource/view.php?id=859317&amp;forceview=1" >Transparencias clase 20 y 27-04-2021. Procesamiento y análisis de imagen</option>
                    <option value="/mod/resource/view.php?id=1342537&amp;forceview=1" >Tareas 27 de abril al 3 de mayo</option>
                    <option value="/mod/assign/view.php?id=1341958&amp;forceview=1" >Entrega tareas 27 abril al 3 de mayo (03-05-2021 23:55)</option>
                    <option value="/mod/resource/view.php?id=1346437&amp;forceview=1" >Tareas 5 al 10 de mayo</option>
                    <option value="/mod/assign/view.php?id=1346431&amp;forceview=1" >Entrega tareas 5 al 10 de mayo (10-05-2021 23:55)</option>
                    <option value="/mod/resource/view.php?id=1104350&amp;forceview=1" >Enunciado entrega final de análisis de imagen</option>
                    <option value="/mod/assign/view.php?id=1341962&amp;forceview=1" >Entrega memoria de análisis de imagen y reconocimiento de marcas (14-05-2021 23:55)</option>
                    <option value="/mod/url/view.php?id=758958&amp;forceview=1" >VideoA</option>
                    <option value="/mod/url/view.php?id=763262&amp;forceview=1" >Video B</option>
                    <option value="/mod/resource/view.php?id=665594&amp;forceview=1" >Video C</option>
                    <option value="/mod/url/view.php?id=550786&amp;forceview=1" >Vídeo 1</option>
                    <option value="/mod/resource/view.php?id=1103690&amp;forceview=1" >Imágenes entrenamiento marcas</option>
                    <option value="/mod/resource/view.php?id=1103688&amp;forceview=1" >Vídeo sintético</option>
                    <option value="/mod/resource/view.php?id=1349338&amp;forceview=1" >Transparencias explicando la integración de los 3 partes y el examen final.</option>
                    <option value="/mod/resource/view.php?id=1345092&amp;forceview=1" >Zip del mundo para la entrega final (v02)</option>
                    <option value="/mod/assign/view.php?id=480592&amp;forceview=1" >Entrega Final (junio)</option>
                    <option value="/mod/assign/view.php?id=531335&amp;forceview=1" >Entrega Final (julio)</option>
        </select>
            <noscript>
                <input type="submit" class="btn btn-secondary ml-1" value="Ir">
            </noscript>
    </form>
</div>

        </div>
</div>
    <div class="col-md-4">        <div class="float-right">
                <a href="https://moodle.upm.es/titulaciones/oficiales/mod/assign/view.php?id=480592&forceview=1" id="next-activity-link" class="btn btn-link"  title="Entrega Final (junio)" >Entrega Final (junio) &#x25BA;</a>

        </div>
</div>
</div>
</div>
                    

                </section>
                <section data-region="blocks-column" class="d-print-none" aria-label="Bloques">
                    <aside id="block-region-side-pre" class="block-region" data-blockregion="side-pre" data-droptarget="1"><a href="#sb-1" class="sr-only sr-only-focusable">Salta Encuesta de satisfacción</a>

<section id="inst2133669"
     class=" block_html block  card mb-3"
     role="complementary"
     data-block="html"
          aria-labelledby="instance-2133669-header"
     >

    <div class="card-body p-3">

            <h5 id="instance-2133669-header" class="card-title d-inline">Encuesta de satisfacción</h5>


        <div class="card-text content mt-3">
            <div class="no-overflow"><p>
<a href="https://moodle.upm.es/titulaciones/oficiales/mod/page/view.php?id=938600" target="_blank">- Encuesta para profesores.</a><br>
<a href="https://www.upm.es/encuesta/encuesta.upm?ec=40" target="_blank">- Encuesta para estudiantes.</a>
<br>
</p></div>
            <div class="footer"></div>
            
        </div>

    </div>

</section>

  <span id="sb-1"></span><a href="#sb-2" class="sr-only sr-only-focusable">Salta UPM</a>

<section id="inst1172166"
     class=" block_html block  card mb-3"
     role="complementary"
     data-block="html"
          aria-labelledby="instance-1172166-header"
     >

    <div class="card-body p-3">

            <h5 id="instance-1172166-header" class="card-title d-inline">UPM</h5>


        <div class="card-text content mt-3">
            <div class="no-overflow">En <b><a href="http://serviciosgate.upm.es/trabajar-desde-casa/" target="_blank"><span class="" style="color: rgb(255, 102, 51); font-size: large;">Trabajar desde casa</span></a></b> puede consultar
los servicios y prestaciones de la UPM para su actividad en remoto.</div>
            <div class="footer"></div>
            
        </div>

    </div>

</section>

  <span id="sb-2"></span><a href="#sb-3" class="sr-only sr-only-focusable">Salta Atención al usuario</a>

<section id="inst355992"
     class=" block_html block  card mb-3"
     role="complementary"
     data-block="html"
          aria-labelledby="instance-355992-header"
     >

    <div class="card-body p-3">

            <h5 id="instance-355992-header" class="card-title d-inline">Atención al usuario</h5>


        <div class="card-text content mt-3">
            <div class="no-overflow"><table>
<thead>
<tr>
<th scope="col" style="text-align: center;"><a href="https://moodle.upm.es/titulaciones/oficiales/mod/page/view.php?id=856744"><img src="https://moodle.upm.es/titulaciones/oficiales/pluginfile.php/1436350/block_html/content/Icono%20de%20teleo-perador.png" alt="Icono de tele-operador" class="img-responsive atto_image_button_middle" width="64" height="64"></a><br></th>
<th scope="col"><a href="https://moodle.upm.es/titulaciones/oficiales/mod/page/view.php?id=867182">Si necesitas ayuda, contacta con nosotros</a><br></th>
</tr>
</thead>
<tbody>
</tbody>
</table></div>
            <div class="footer"></div>
            
        </div>

    </div>

</section>

  <span id="sb-3"></span></aside>
                </section>
            </div>
        </div>
    </div>
    <div
    id="drawer-60bf3510d20bb60bf3510cb0c518"
    class=" drawer bg-white hidden"
    aria-expanded="false"
    aria-hidden="true"
    data-region="right-hand-drawer"
    role="region"
    tabindex="0"
>
            <div id="message-drawer-60bf3510d20bb60bf3510cb0c518" class="message-app" data-region="message-drawer" role="region">
            <div class="closewidget bg-light border-bottom text-right">
                <a class="text-dark" data-action="closedrawer" href="#"
                   title="Cerrar" aria-label="Cerrar"
                >
                    <i class="icon fa fa-window-close fa-fw " aria-hidden="true"  ></i>
                </a>
            </div>
            <div class="header-container position-relative" data-region="header-container">
                <div class="hidden border-bottom px-2 py-3" aria-hidden="true" data-region="view-contacts">
                    <div class="d-flex align-items-center">
                        <div class="align-self-stretch">
                            <a class="h-100 d-flex align-items-center mr-2" href="#" data-route-back role="button">
                                <div class="icon-back-in-drawer">
                                    <span class="dir-rtl-hide"><i class="icon fa fa-chevron-left fa-fw " aria-hidden="true"  ></i></span>
                                    <span class="dir-ltr-hide"><i class="icon fa fa-chevron-right fa-fw " aria-hidden="true"  ></i></span>
                                </div>
                                <div class="icon-back-in-app">
                                    <span class="dir-rtl-hide"><i class="icon fa fa-times fa-fw " aria-hidden="true"  ></i></span>
                                </div>                            </a>
                        </div>
                        <div>
                            Contactos
                        </div>
                        <div class="ml-auto">
                            <a href="#" data-route="view-search" role="button" aria-label="Búsqueda">
                                <i class="icon fa fa-search fa-fw " aria-hidden="true"  ></i>
                            </a>
                        </div>
                    </div>
                </div>                
                <div
                    class="hidden bg-white position-relative border-bottom p-1 p-sm-2"
                    aria-hidden="true"
                    data-region="view-conversation"
                >
                    <div class="hidden" data-region="header-content"></div>
                    <div class="hidden" data-region="header-edit-mode">
                        
                        <div class="d-flex p-2 align-items-center">
                            Mensajes seleccionados:
                            <span class="ml-1" data-region="message-selected-court">1</span>
                            <button type="button" class="ml-auto close" aria-label="Cancelar selección de mensaje"
                                data-action="cancel-edit-mode">
                                    <span aria-hidden="true">&times;</span>
                            </button>
                        </div>
                    </div>
                    <div data-region="header-placeholder">
                        <div class="d-flex">
                            <div
                                class="ml-2 rounded-circle bg-pulse-grey align-self-center"
                                style="height: 38px; width: 38px"
                            >
                            </div>
                            <div class="ml-2 " style="flex: 1">
                                <div
                                    class="mt-1 bg-pulse-grey w-75"
                                    style="height: 16px;"
                                >
                                </div>
                            </div>
                            <div
                                class="ml-2 bg-pulse-grey align-self-center"
                                style="height: 16px; width: 20px"
                            >
                            </div>
                        </div>
                    </div>
                    <div
                        class="hidden position-absolute"
                        data-region="confirm-dialogue-container"
                        style="top: 0; bottom: -1px; right: 0; left: 0; background: rgba(0,0,0,0.3);"
                    ></div>
                </div>                <div class="border-bottom  p-1 px-sm-2 py-sm-3" aria-hidden="false"  data-region="view-overview">
                    <div class="d-flex align-items-center">
                        <div class="input-group">
                            <div class="input-group-prepend">
                                <span class="input-group-text pr-2 bg-white">
                                    <i class="icon fa fa-search fa-fw " aria-hidden="true"  ></i>
                                </span>
                            </div>
                            <input
                                type="text"
                                class="form-control border-left-0"
                                placeholder="Búsqueda"
                                aria-label="Búsqueda"
                                data-region="view-overview-search-input"
                            >
                        </div>
                        <div class="ml-2">
                            <a
                                href="#"
                                data-route="view-settings"
                                data-route-param="18361061"
                                aria-label="Ajustes"
                                role="button"
                            >
                                <i class="icon fa fa-cog fa-fw " aria-hidden="true"  ></i>
                            </a>
                        </div>
                    </div>
                    <div class="text-right mt-sm-3">
                        <a href="#" data-route="view-contacts" role="button">
                            <i class="icon fa fa-user fa-fw " aria-hidden="true"  ></i>
                            Contactos
                            <span class="badge badge-primary bg-primary ml-2 hidden"
                            data-region="contact-request-count"
                            aria-label="There are 0 pending contact requests">
                                0
                            </span>
                        </a>
                    </div>
                </div>
                
                <div class="hidden border-bottom px-2 py-3 view-search"  aria-hidden="true" data-region="view-search">
                    <div class="d-flex align-items-center">
                        <a
                            class="mr-2 align-self-stretch d-flex align-items-center"
                            href="#"
                            data-route-back
                            data-action="cancel-search"
                            role="button"
                        >
                            <div class="icon-back-in-drawer">
                                <span class="dir-rtl-hide"><i class="icon fa fa-chevron-left fa-fw " aria-hidden="true"  ></i></span>
                                <span class="dir-ltr-hide"><i class="icon fa fa-chevron-right fa-fw " aria-hidden="true"  ></i></span>
                            </div>
                            <div class="icon-back-in-app">
                                <span class="dir-rtl-hide"><i class="icon fa fa-times fa-fw " aria-hidden="true"  ></i></span>
                            </div>                        </a>
                        <div class="input-group">
                            <input
                                type="text"
                                class="form-control"
                                placeholder="Búsqueda"
                                aria-label="Búsqueda"
                                data-region="search-input"
                            >
                            <div class="input-group-append">
                                <button
                                    class="btn btn-outline-secondary"
                                    type="button"
                                    data-action="search"
                                    aria-label="Búsqueda"
                                >
                                    <span data-region="search-icon-container">
                                        <i class="icon fa fa-search fa-fw " aria-hidden="true"  ></i>
                                    </span>
                                    <span class="hidden" data-region="loading-icon-container">
                                        <span class="loading-icon icon-no-margin"><i class="icon fa fa-circle-o-notch fa-spin fa-fw "  title="Enviando" aria-label="Enviando"></i></span>
                                    </span>
                                </button>
                            </div>
                        </div>
                    </div>
                </div>                
                <div class="hidden border-bottom px-2 py-3" aria-hidden="true" data-region="view-settings">
                    <div class="d-flex align-items-center">
                        <div class="align-self-stretch" >
                            <a class="h-100 d-flex mr-2 align-items-center" href="#" data-route-back role="button">
                                <div class="icon-back-in-drawer">
                                    <span class="dir-rtl-hide"><i class="icon fa fa-chevron-left fa-fw " aria-hidden="true"  ></i></span>
                                    <span class="dir-ltr-hide"><i class="icon fa fa-chevron-right fa-fw " aria-hidden="true"  ></i></span>
                                </div>
                                <div class="icon-back-in-app">
                                    <span class="dir-rtl-hide"><i class="icon fa fa-times fa-fw " aria-hidden="true"  ></i></span>
                                </div>                            </a>
                        </div>
                        <div>
                            Configuración
                        </div>
                    </div>
                </div>
            </div>
            <div class="body-container position-relative" data-region="body-container">
                
                <div
                    class="hidden"
                    data-region="view-contact"
                    aria-hidden="true"
                >
                    <div class="p-2 pt-3" data-region="content-container"></div>
                </div>                <div class="hidden h-100" data-region="view-contacts" aria-hidden="true" data-user-id="18361061">
                    <div class="d-flex flex-column h-100">
                        <div class="p-3 border-bottom">
                            <ul class="nav nav-pills nav-fill" role="tablist">
                                <li class="nav-item">
                                    <a
                                        id="contacts-tab-60bf3510d20bb60bf3510cb0c518"
                                        class="nav-link active"
                                        href="#contacts-tab-panel-60bf3510d20bb60bf3510cb0c518"
                                        data-toggle="tab"
                                        data-action="show-contacts-section"
                                        role="tab"
                                        aria-controls="contacts-tab-panel-60bf3510d20bb60bf3510cb0c518"
                                        aria-selected="true"
                                    >
                                        Contactos
                                    </a>
                                </li>
                                <li class="nav-item">
                                    <a
                                        id="requests-tab-60bf3510d20bb60bf3510cb0c518"
                                        class="nav-link"
                                        href="#requests-tab-panel-60bf3510d20bb60bf3510cb0c518"
                                        data-toggle="tab"
                                        data-action="show-requests-section"
                                        role="tab"
                                        aria-controls="requests-tab-panel-60bf3510d20bb60bf3510cb0c518"
                                        aria-selected="false"
                                    >
                                        Peticiones
                                        <span class="badge badge-primary bg-primary ml-2 hidden"
                                        data-region="contact-request-count"
                                        aria-label="There are 0 pending contact requests">
                                            0
                                        </span>
                                    </a>
                                </li>
                            </ul>
                        </div>
                        <div class="tab-content d-flex flex-column h-100">
                                            <div
                    class="tab-pane fade show active h-100 lazy-load-list"
                    aria-live="polite"
                    data-region="lazy-load-list"
                    data-user-id="18361061"
                                        id="contacts-tab-panel-60bf3510d20bb60bf3510cb0c518"
                    data-section="contacts"
                    role="tabpanel"
                    aria-labelledby="contacts-tab-60bf3510d20bb60bf3510cb0c518"

                >
                    
                    <div class="hidden text-center p-2" data-region="empty-message-container">
                        No hay contactos
                    </div>
                    <div class="hidden list-group" data-region="content-container">
                        
                    </div>
                    <div class="list-group" data-region="placeholder-container">
                        
                    </div>
                    <div class="w-100 text-center p-3 hidden" data-region="loading-icon-container" >
                        <span class="loading-icon icon-no-margin"><i class="icon fa fa-circle-o-notch fa-spin fa-fw "  title="Enviando" aria-label="Enviando"></i></span>
                    </div>
                </div>
                
                                            <div
                    class="tab-pane fade h-100 lazy-load-list"
                    aria-live="polite"
                    data-region="lazy-load-list"
                    data-user-id="18361061"
                                        id="requests-tab-panel-60bf3510d20bb60bf3510cb0c518"
                    data-section="requests"
                    role="tabpanel"
                    aria-labelledby="requests-tab-60bf3510d20bb60bf3510cb0c518"

                >
                    
                    <div class="hidden text-center p-2" data-region="empty-message-container">
                        No hay solicitudes de contacto
                    </div>
                    <div class="hidden list-group" data-region="content-container">
                        
                    </div>
                    <div class="list-group" data-region="placeholder-container">
                        
                    </div>
                    <div class="w-100 text-center p-3 hidden" data-region="loading-icon-container" >
                        <span class="loading-icon icon-no-margin"><i class="icon fa fa-circle-o-notch fa-spin fa-fw "  title="Enviando" aria-label="Enviando"></i></span>
                    </div>
                </div>
                        </div>
                    </div>
                </div>                
                <div
                    class="view-conversation hidden h-100"
                    aria-hidden="true"
                    data-region="view-conversation"
                    data-user-id="18361061"
                    data-midnight="1623103200"
                    data-message-poll-min="10"
                    data-message-poll-max="120"
                    data-message-poll-after-max="300"
                    style="overflow-y: auto; overflow-x: hidden"
                >
                    <div class="position-relative h-100" data-region="content-container" style="overflow-y: auto; overflow-x: hidden">
                        <div class="content-message-container hidden h-100 px-2 pt-0" data-region="content-message-container" role="log" style="overflow-y: auto; overflow-x: hidden">
                            <div class="py-3 sticky-top z-index-1 border-bottom text-center hidden" data-region="contact-request-sent-message-container">
                                <p class="m-0">Solicitud de contacto enviada</p>
                                <p class="font-italic font-weight-light" data-region="text"></p>
                            </div>
                            <div class="p-3 text-center hidden" data-region="self-conversation-message-container">
                                <p class="m-0">Espacio personal</p>
                                <p class="font-italic font-weight-light" data-region="text">Guardar borradores de mensajes, notas, etc, para acceder a ellos más tarde.</p>
                           </div>
                            <div class="hidden text-center p-3" data-region="more-messages-loading-icon-container"><span class="loading-icon icon-no-margin"><i class="icon fa fa-circle-o-notch fa-spin fa-fw "  title="Enviando" aria-label="Enviando"></i></span>
</div>
                        </div>
                        <div class="p-4 w-100 h-100 hidden position-absolute" data-region="confirm-dialogue-container" style="top: 0; background: rgba(0,0,0,0.3);">
                            
                            <div class="p-3 bg-white" data-region="confirm-dialogue" role="alert">
                                <p class="text-muted" data-region="dialogue-text"></p>
                                <div class="mb-2 custom-control custom-checkbox hidden" data-region="delete-messages-for-all-users-toggle-container">
                                    <input type="checkbox" class="custom-control-input" id="delete-messages-for-all-users" data-region="delete-messages-for-all-users-toggle">
                                    <label class="custom-control-label text-muted" for="delete-messages-for-all-users">
                                        Borrar para mí y para todos los demás
                                    </label>
                                </div>
                                <button type="button" class="btn btn-primary btn-block hidden" data-action="confirm-block">
                                    <span data-region="dialogue-button-text">Bloque</span>
                                    <span class="hidden" data-region="loading-icon-container"><span class="loading-icon icon-no-margin"><i class="icon fa fa-circle-o-notch fa-spin fa-fw "  title="Enviando" aria-label="Enviando"></i></span>
</span>
                                </button>
                                <button type="button" class="btn btn-primary btn-block hidden" data-action="confirm-unblock">
                                    <span data-region="dialogue-button-text">Desbloquear</span>
                                    <span class="hidden" data-region="loading-icon-container"><span class="loading-icon icon-no-margin"><i class="icon fa fa-circle-o-notch fa-spin fa-fw "  title="Enviando" aria-label="Enviando"></i></span>
</span>
                                </button>
                                <button type="button" class="btn btn-primary btn-block hidden" data-action="confirm-remove-contact">
                                    <span data-region="dialogue-button-text">Quitar</span>
                                    <span class="hidden" data-region="loading-icon-container"><span class="loading-icon icon-no-margin"><i class="icon fa fa-circle-o-notch fa-spin fa-fw "  title="Enviando" aria-label="Enviando"></i></span>
</span>
                                </button>
                                <button type="button" class="btn btn-primary btn-block hidden" data-action="confirm-add-contact">
                                    <span data-region="dialogue-button-text">Agregar</span>
                                    <span class="hidden" data-region="loading-icon-container"><span class="loading-icon icon-no-margin"><i class="icon fa fa-circle-o-notch fa-spin fa-fw "  title="Enviando" aria-label="Enviando"></i></span>
</span>
                                </button>
                                <button type="button" class="btn btn-primary btn-block hidden" data-action="confirm-delete-selected-messages">
                                    <span data-region="dialogue-button-text">Borrar</span>
                                    <span class="hidden" data-region="loading-icon-container"><span class="loading-icon icon-no-margin"><i class="icon fa fa-circle-o-notch fa-spin fa-fw "  title="Enviando" aria-label="Enviando"></i></span>
</span>
                                </button>
                                <button type="button" class="btn btn-primary btn-block hidden" data-action="confirm-delete-conversation">
                                    <span data-region="dialogue-button-text">Borrar</span>
                                    <span class="hidden" data-region="loading-icon-container"><span class="loading-icon icon-no-margin"><i class="icon fa fa-circle-o-notch fa-spin fa-fw "  title="Enviando" aria-label="Enviando"></i></span>
</span>
                                </button>
                                <button type="button" class="btn btn-primary btn-block hidden" data-action="request-add-contact">
                                    <span data-region="dialogue-button-text">Enviar solicitud de contacto</span>
                                    <span class="hidden" data-region="loading-icon-container"><span class="loading-icon icon-no-margin"><i class="icon fa fa-circle-o-notch fa-spin fa-fw "  title="Enviando" aria-label="Enviando"></i></span>
</span>
                                </button>
                                <button type="button" class="btn btn-primary btn-block hidden" data-action="accept-contact-request">
                                    <span data-region="dialogue-button-text">Aceptar y añadir a los contactos</span>
                                    <span class="hidden" data-region="loading-icon-container"><span class="loading-icon icon-no-margin"><i class="icon fa fa-circle-o-notch fa-spin fa-fw "  title="Enviando" aria-label="Enviando"></i></span>
</span>
                                </button>
                                <button type="button" class="btn btn-secondary btn-block hidden" data-action="decline-contact-request">
                                    <span data-region="dialogue-button-text">Rechazar</span>
                                    <span class="hidden" data-region="loading-icon-container"><span class="loading-icon icon-no-margin"><i class="icon fa fa-circle-o-notch fa-spin fa-fw "  title="Enviando" aria-label="Enviando"></i></span>
</span>
                                </button>
                                <button type="button" class="btn btn-primary btn-block" data-action="okay-confirm">OK</button>
                                <button type="button" class="btn btn-secondary btn-block" data-action="cancel-confirm">Cancelar</button>
                            </div>
                        </div>
                        <div class="px-2 pb-2 pt-0" data-region="content-placeholder">
                            <div class="h-100 d-flex flex-column">
                                <div
                                    class="px-2 pb-2 pt-0 bg-light h-100"
                                    style="overflow-y: auto"
                                >
                                    <div class="mt-4">
                                        <div class="mb-4">
                                            <div class="mx-auto bg-white" style="height: 25px; width: 100px"></div>
                                        </div>
                                        <div class="d-flex flex-column p-2 bg-white rounded mb-2">
                                            <div class="d-flex align-items-center mb-2">
                                                <div class="mr-2">
                                                    <div class="rounded-circle bg-pulse-grey" style="height: 35px; width: 35px"></div>
                                                </div>
                                                <div class="mr-4 w-75 bg-pulse-grey" style="height: 16px"></div>
                                                <div class="ml-auto bg-pulse-grey" style="width: 35px; height: 16px"></div>
                                            </div>
                                            <div class="bg-pulse-grey w-100" style="height: 16px"></div>
                                            <div class="bg-pulse-grey w-100 mt-2" style="height: 16px"></div>
                                            <div class="bg-pulse-grey w-100 mt-2" style="height: 16px"></div>
                                            <div class="bg-pulse-grey w-100 mt-2" style="height: 16px"></div>
                                            <div class="bg-pulse-grey w-75 mt-2" style="height: 16px"></div>
                                        </div>
                                        <div class="d-flex flex-column p-2 bg-white rounded mb-2">
                                            <div class="d-flex align-items-center mb-2">
                                                <div class="mr-2">
                                                    <div class="rounded-circle bg-pulse-grey" style="height: 35px; width: 35px"></div>
                                                </div>
                                                <div class="mr-4 w-75 bg-pulse-grey" style="height: 16px"></div>
                                                <div class="ml-auto bg-pulse-grey" style="width: 35px; height: 16px"></div>
                                            </div>
                                            <div class="bg-pulse-grey w-100" style="height: 16px"></div>
                                            <div class="bg-pulse-grey w-100 mt-2" style="height: 16px"></div>
                                            <div class="bg-pulse-grey w-100 mt-2" style="height: 16px"></div>
                                            <div class="bg-pulse-grey w-100 mt-2" style="height: 16px"></div>
                                            <div class="bg-pulse-grey w-75 mt-2" style="height: 16px"></div>
                                        </div>
                                        <div class="d-flex flex-column p-2 bg-white rounded mb-2">
                                            <div class="d-flex align-items-center mb-2">
                                                <div class="mr-2">
                                                    <div class="rounded-circle bg-pulse-grey" style="height: 35px; width: 35px"></div>
                                                </div>
                                                <div class="mr-4 w-75 bg-pulse-grey" style="height: 16px"></div>
                                                <div class="ml-auto bg-pulse-grey" style="width: 35px; height: 16px"></div>
                                            </div>
                                            <div class="bg-pulse-grey w-100" style="height: 16px"></div>
                                            <div class="bg-pulse-grey w-100 mt-2" style="height: 16px"></div>
                                            <div class="bg-pulse-grey w-100 mt-2" style="height: 16px"></div>
                                            <div class="bg-pulse-grey w-100 mt-2" style="height: 16px"></div>
                                            <div class="bg-pulse-grey w-75 mt-2" style="height: 16px"></div>
                                        </div>
                                    </div>                                    <div class="mt-4">
                                        <div class="mb-4">
                                            <div class="mx-auto bg-white" style="height: 25px; width: 100px"></div>
                                        </div>
                                        <div class="d-flex flex-column p-2 bg-white rounded mb-2">
                                            <div class="d-flex align-items-center mb-2">
                                                <div class="mr-2">
                                                    <div class="rounded-circle bg-pulse-grey" style="height: 35px; width: 35px"></div>
                                                </div>
                                                <div class="mr-4 w-75 bg-pulse-grey" style="height: 16px"></div>
                                                <div class="ml-auto bg-pulse-grey" style="width: 35px; height: 16px"></div>
                                            </div>
                                            <div class="bg-pulse-grey w-100" style="height: 16px"></div>
                                            <div class="bg-pulse-grey w-100 mt-2" style="height: 16px"></div>
                                            <div class="bg-pulse-grey w-100 mt-2" style="height: 16px"></div>
                                            <div class="bg-pulse-grey w-100 mt-2" style="height: 16px"></div>
                                            <div class="bg-pulse-grey w-75 mt-2" style="height: 16px"></div>
                                        </div>
                                        <div class="d-flex flex-column p-2 bg-white rounded mb-2">
                                            <div class="d-flex align-items-center mb-2">
                                                <div class="mr-2">
                                                    <div class="rounded-circle bg-pulse-grey" style="height: 35px; width: 35px"></div>
                                                </div>
                                                <div class="mr-4 w-75 bg-pulse-grey" style="height: 16px"></div>
                                                <div class="ml-auto bg-pulse-grey" style="width: 35px; height: 16px"></div>
                                            </div>
                                            <div class="bg-pulse-grey w-100" style="height: 16px"></div>
                                            <div class="bg-pulse-grey w-100 mt-2" style="height: 16px"></div>
                                            <div class="bg-pulse-grey w-100 mt-2" style="height: 16px"></div>
                                            <div class="bg-pulse-grey w-100 mt-2" style="height: 16px"></div>
                                            <div class="bg-pulse-grey w-75 mt-2" style="height: 16px"></div>
                                        </div>
                                        <div class="d-flex flex-column p-2 bg-white rounded mb-2">
                                            <div class="d-flex align-items-center mb-2">
                                                <div class="mr-2">
                                                    <div class="rounded-circle bg-pulse-grey" style="height: 35px; width: 35px"></div>
                                                </div>
                                                <div class="mr-4 w-75 bg-pulse-grey" style="height: 16px"></div>
                                                <div class="ml-auto bg-pulse-grey" style="width: 35px; height: 16px"></div>
                                            </div>
                                            <div class="bg-pulse-grey w-100" style="height: 16px"></div>
                                            <div class="bg-pulse-grey w-100 mt-2" style="height: 16px"></div>
                                            <div class="bg-pulse-grey w-100 mt-2" style="height: 16px"></div>
                                            <div class="bg-pulse-grey w-100 mt-2" style="height: 16px"></div>
                                            <div class="bg-pulse-grey w-75 mt-2" style="height: 16px"></div>
                                        </div>
                                    </div>                                    <div class="mt-4">
                                        <div class="mb-4">
                                            <div class="mx-auto bg-white" style="height: 25px; width: 100px"></div>
                                        </div>
                                        <div class="d-flex flex-column p-2 bg-white rounded mb-2">
                                            <div class="d-flex align-items-center mb-2">
                                                <div class="mr-2">
                                                    <div class="rounded-circle bg-pulse-grey" style="height: 35px; width: 35px"></div>
                                                </div>
                                                <div class="mr-4 w-75 bg-pulse-grey" style="height: 16px"></div>
                                                <div class="ml-auto bg-pulse-grey" style="width: 35px; height: 16px"></div>
                                            </div>
                                            <div class="bg-pulse-grey w-100" style="height: 16px"></div>
                                            <div class="bg-pulse-grey w-100 mt-2" style="height: 16px"></div>
                                            <div class="bg-pulse-grey w-100 mt-2" style="height: 16px"></div>
                                            <div class="bg-pulse-grey w-100 mt-2" style="height: 16px"></div>
                                            <div class="bg-pulse-grey w-75 mt-2" style="height: 16px"></div>
                                        </div>
                                        <div class="d-flex flex-column p-2 bg-white rounded mb-2">
                                            <div class="d-flex align-items-center mb-2">
                                                <div class="mr-2">
                                                    <div class="rounded-circle bg-pulse-grey" style="height: 35px; width: 35px"></div>
                                                </div>
                                                <div class="mr-4 w-75 bg-pulse-grey" style="height: 16px"></div>
                                                <div class="ml-auto bg-pulse-grey" style="width: 35px; height: 16px"></div>
                                            </div>
                                            <div class="bg-pulse-grey w-100" style="height: 16px"></div>
                                            <div class="bg-pulse-grey w-100 mt-2" style="height: 16px"></div>
                                            <div class="bg-pulse-grey w-100 mt-2" style="height: 16px"></div>
                                            <div class="bg-pulse-grey w-100 mt-2" style="height: 16px"></div>
                                            <div class="bg-pulse-grey w-75 mt-2" style="height: 16px"></div>
                                        </div>
                                        <div class="d-flex flex-column p-2 bg-white rounded mb-2">
                                            <div class="d-flex align-items-center mb-2">
                                                <div class="mr-2">
                                                    <div class="rounded-circle bg-pulse-grey" style="height: 35px; width: 35px"></div>
                                                </div>
                                                <div class="mr-4 w-75 bg-pulse-grey" style="height: 16px"></div>
                                                <div class="ml-auto bg-pulse-grey" style="width: 35px; height: 16px"></div>
                                            </div>
                                            <div class="bg-pulse-grey w-100" style="height: 16px"></div>
                                            <div class="bg-pulse-grey w-100 mt-2" style="height: 16px"></div>
                                            <div class="bg-pulse-grey w-100 mt-2" style="height: 16px"></div>
                                            <div class="bg-pulse-grey w-100 mt-2" style="height: 16px"></div>
                                            <div class="bg-pulse-grey w-75 mt-2" style="height: 16px"></div>
                                        </div>
                                    </div>                                    <div class="mt-4">
                                        <div class="mb-4">
                                            <div class="mx-auto bg-white" style="height: 25px; width: 100px"></div>
                                        </div>
                                        <div class="d-flex flex-column p-2 bg-white rounded mb-2">
                                            <div class="d-flex align-items-center mb-2">
                                                <div class="mr-2">
                                                    <div class="rounded-circle bg-pulse-grey" style="height: 35px; width: 35px"></div>
                                                </div>
                                                <div class="mr-4 w-75 bg-pulse-grey" style="height: 16px"></div>
                                                <div class="ml-auto bg-pulse-grey" style="width: 35px; height: 16px"></div>
                                            </div>
                                            <div class="bg-pulse-grey w-100" style="height: 16px"></div>
                                            <div class="bg-pulse-grey w-100 mt-2" style="height: 16px"></div>
                                            <div class="bg-pulse-grey w-100 mt-2" style="height: 16px"></div>
                                            <div class="bg-pulse-grey w-100 mt-2" style="height: 16px"></div>
                                            <div class="bg-pulse-grey w-75 mt-2" style="height: 16px"></div>
                                        </div>
                                        <div class="d-flex flex-column p-2 bg-white rounded mb-2">
                                            <div class="d-flex align-items-center mb-2">
                                                <div class="mr-2">
                                                    <div class="rounded-circle bg-pulse-grey" style="height: 35px; width: 35px"></div>
                                                </div>
                                                <div class="mr-4 w-75 bg-pulse-grey" style="height: 16px"></div>
                                                <div class="ml-auto bg-pulse-grey" style="width: 35px; height: 16px"></div>
                                            </div>
                                            <div class="bg-pulse-grey w-100" style="height: 16px"></div>
                                            <div class="bg-pulse-grey w-100 mt-2" style="height: 16px"></div>
                                            <div class="bg-pulse-grey w-100 mt-2" style="height: 16px"></div>
                                            <div class="bg-pulse-grey w-100 mt-2" style="height: 16px"></div>
                                            <div class="bg-pulse-grey w-75 mt-2" style="height: 16px"></div>
                                        </div>
                                        <div class="d-flex flex-column p-2 bg-white rounded mb-2">
                                            <div class="d-flex align-items-center mb-2">
                                                <div class="mr-2">
                                                    <div class="rounded-circle bg-pulse-grey" style="height: 35px; width: 35px"></div>
                                                </div>
                                                <div class="mr-4 w-75 bg-pulse-grey" style="height: 16px"></div>
                                                <div class="ml-auto bg-pulse-grey" style="width: 35px; height: 16px"></div>
                                            </div>
                                            <div class="bg-pulse-grey w-100" style="height: 16px"></div>
                                            <div class="bg-pulse-grey w-100 mt-2" style="height: 16px"></div>
                                            <div class="bg-pulse-grey w-100 mt-2" style="height: 16px"></div>
                                            <div class="bg-pulse-grey w-100 mt-2" style="height: 16px"></div>
                                            <div class="bg-pulse-grey w-75 mt-2" style="height: 16px"></div>
                                        </div>
                                    </div>                                    <div class="mt-4">
                                        <div class="mb-4">
                                            <div class="mx-auto bg-white" style="height: 25px; width: 100px"></div>
                                        </div>
                                        <div class="d-flex flex-column p-2 bg-white rounded mb-2">
                                            <div class="d-flex align-items-center mb-2">
                                                <div class="mr-2">
                                                    <div class="rounded-circle bg-pulse-grey" style="height: 35px; width: 35px"></div>
                                                </div>
                                                <div class="mr-4 w-75 bg-pulse-grey" style="height: 16px"></div>
                                                <div class="ml-auto bg-pulse-grey" style="width: 35px; height: 16px"></div>
                                            </div>
                                            <div class="bg-pulse-grey w-100" style="height: 16px"></div>
                                            <div class="bg-pulse-grey w-100 mt-2" style="height: 16px"></div>
                                            <div class="bg-pulse-grey w-100 mt-2" style="height: 16px"></div>
                                            <div class="bg-pulse-grey w-100 mt-2" style="height: 16px"></div>
                                            <div class="bg-pulse-grey w-75 mt-2" style="height: 16px"></div>
                                        </div>
                                        <div class="d-flex flex-column p-2 bg-white rounded mb-2">
                                            <div class="d-flex align-items-center mb-2">
                                                <div class="mr-2">
                                                    <div class="rounded-circle bg-pulse-grey" style="height: 35px; width: 35px"></div>
                                                </div>
                                                <div class="mr-4 w-75 bg-pulse-grey" style="height: 16px"></div>
                                                <div class="ml-auto bg-pulse-grey" style="width: 35px; height: 16px"></div>
                                            </div>
                                            <div class="bg-pulse-grey w-100" style="height: 16px"></div>
                                            <div class="bg-pulse-grey w-100 mt-2" style="height: 16px"></div>
                                            <div class="bg-pulse-grey w-100 mt-2" style="height: 16px"></div>
                                            <div class="bg-pulse-grey w-100 mt-2" style="height: 16px"></div>
                                            <div class="bg-pulse-grey w-75 mt-2" style="height: 16px"></div>
                                        </div>
                                        <div class="d-flex flex-column p-2 bg-white rounded mb-2">
                                            <div class="d-flex align-items-center mb-2">
                                                <div class="mr-2">
                                                    <div class="rounded-circle bg-pulse-grey" style="height: 35px; width: 35px"></div>
                                                </div>
                                                <div class="mr-4 w-75 bg-pulse-grey" style="height: 16px"></div>
                                                <div class="ml-auto bg-pulse-grey" style="width: 35px; height: 16px"></div>
                                            </div>
                                            <div class="bg-pulse-grey w-100" style="height: 16px"></div>
                                            <div class="bg-pulse-grey w-100 mt-2" style="height: 16px"></div>
                                            <div class="bg-pulse-grey w-100 mt-2" style="height: 16px"></div>
                                            <div class="bg-pulse-grey w-100 mt-2" style="height: 16px"></div>
                                            <div class="bg-pulse-grey w-75 mt-2" style="height: 16px"></div>
                                        </div>
                                    </div>                                </div>
                            </div>                        </div>
                    </div>
                </div>
                
                <div
                    class="hidden"
                    aria-hidden="true"
                    data-region="view-group-info"
                >
                    <div
                        class="pt-3 h-100 d-flex flex-column"
                        data-region="group-info-content-container"
                        style="overflow-y: auto"
                    ></div>
                </div>                <div class="h-100 view-overview-body" aria-hidden="false" data-region="view-overview"  data-user-id="18361061">
                    <div id="message-drawer-view-overview-container-60bf3510d20bb60bf3510cb0c518" class="d-flex flex-column h-100" style="overflow-y: auto">
                            
                            
                            <div
                                class="section border-0 card"
                                data-region="view-overview-favourites"
                            >
                                <div id="view-overview-favourites-toggle" class="card-header p-0" data-region="toggle">
                                    <button
                                        class="btn btn-link w-100 text-left p-1 p-sm-2 d-flex align-items-center overview-section-toggle collapsed"
                                        data-toggle="collapse"
                                        data-target="#view-overview-favourites-target-60bf3510d20bb60bf3510cb0c518"
                                        aria-expanded="false"
                                        aria-controls="view-overview-favourites-target-60bf3510d20bb60bf3510cb0c518"
                                    >
                                        <span class="collapsed-icon-container">
                                            <i class="icon fa fa-caret-right fa-fw " aria-hidden="true"  ></i>
                                        </span>
                                        <span class="expanded-icon-container">
                                            <i class="icon fa fa-caret-down fa-fw " aria-hidden="true"  ></i>
                                        </span>
                                        <span class="font-weight-bold">Destacados</span>
                                        <small class="hidden ml-1" data-region="section-total-count-container"
                                        aria-label=" total conversations">
                                            (<span data-region="section-total-count"></span>)
                                        </small>
                                        <span class="hidden ml-2" data-region="loading-icon-container">
                                            <span class="loading-icon icon-no-margin"><i class="icon fa fa-circle-o-notch fa-spin fa-fw "  title="Enviando" aria-label="Enviando"></i></span>
                                        </span>
                                        <span class="hidden badge badge-pill badge-primary ml-auto bg-primary"
                                        data-region="section-unread-count"
                                        >
                                            
                                        </span>
                                    </button>
                                </div>
                                                            <div
                                class="collapse border-bottom  lazy-load-list"
                                aria-live="polite"
                                data-region="lazy-load-list"
                                data-user-id="18361061"
                                            id="view-overview-favourites-target-60bf3510d20bb60bf3510cb0c518"
            aria-labelledby="view-overview-favourites-toggle"
            data-parent="#message-drawer-view-overview-container-60bf3510d20bb60bf3510cb0c518"

                            >
                                
                                <div class="hidden text-center p-2" data-region="empty-message-container">
                                            <p class="text-muted mt-2">Sin conversaciones destacadas</p>

                                </div>
                                <div class="hidden list-group" data-region="content-container">
                                    
                                </div>
                                <div class="list-group" data-region="placeholder-container">
                                            <div class="text-center py-2"><span class="loading-icon icon-no-margin"><i class="icon fa fa-circle-o-notch fa-spin fa-fw "  title="Enviando" aria-label="Enviando"></i></span>
</div>

                                </div>
                                <div class="w-100 text-center p-3 hidden" data-region="loading-icon-container" >
                                    <span class="loading-icon icon-no-margin"><i class="icon fa fa-circle-o-notch fa-spin fa-fw "  title="Enviando" aria-label="Enviando"></i></span>
                                </div>
                            </div>
                            </div>
                            
                            
                            <div
                                class="section border-0 card"
                                data-region="view-overview-group-messages"
                            >
                                <div id="view-overview-group-messages-toggle" class="card-header p-0" data-region="toggle">
                                    <button
                                        class="btn btn-link w-100 text-left p-1 p-sm-2 d-flex align-items-center overview-section-toggle collapsed"
                                        data-toggle="collapse"
                                        data-target="#view-overview-group-messages-target-60bf3510d20bb60bf3510cb0c518"
                                        aria-expanded="false"
                                        aria-controls="view-overview-group-messages-target-60bf3510d20bb60bf3510cb0c518"
                                    >
                                        <span class="collapsed-icon-container">
                                            <i class="icon fa fa-caret-right fa-fw " aria-hidden="true"  ></i>
                                        </span>
                                        <span class="expanded-icon-container">
                                            <i class="icon fa fa-caret-down fa-fw " aria-hidden="true"  ></i>
                                        </span>
                                        <span class="font-weight-bold">Grupo</span>
                                        <small class="hidden ml-1" data-region="section-total-count-container"
                                        aria-label=" total conversations">
                                            (<span data-region="section-total-count"></span>)
                                        </small>
                                        <span class="hidden ml-2" data-region="loading-icon-container">
                                            <span class="loading-icon icon-no-margin"><i class="icon fa fa-circle-o-notch fa-spin fa-fw "  title="Enviando" aria-label="Enviando"></i></span>
                                        </span>
                                        <span class="hidden badge badge-pill badge-primary ml-auto bg-primary"
                                        data-region="section-unread-count"
                                        >
                                            
                                        </span>
                                    </button>
                                </div>
                                                            <div
                                class="collapse border-bottom  lazy-load-list"
                                aria-live="polite"
                                data-region="lazy-load-list"
                                data-user-id="18361061"
                                            id="view-overview-group-messages-target-60bf3510d20bb60bf3510cb0c518"
            aria-labelledby="view-overview-group-messages-toggle"
            data-parent="#message-drawer-view-overview-container-60bf3510d20bb60bf3510cb0c518"

                            >
                                
                                <div class="hidden text-center p-2" data-region="empty-message-container">
                                            <p class="text-muted mt-2">No hay conversaciónes de grupo</p>

                                </div>
                                <div class="hidden list-group" data-region="content-container">
                                    
                                </div>
                                <div class="list-group" data-region="placeholder-container">
                                            <div class="text-center py-2"><span class="loading-icon icon-no-margin"><i class="icon fa fa-circle-o-notch fa-spin fa-fw "  title="Enviando" aria-label="Enviando"></i></span>
</div>

                                </div>
                                <div class="w-100 text-center p-3 hidden" data-region="loading-icon-container" >
                                    <span class="loading-icon icon-no-margin"><i class="icon fa fa-circle-o-notch fa-spin fa-fw "  title="Enviando" aria-label="Enviando"></i></span>
                                </div>
                            </div>
                            </div>
                            
                            
                            <div
                                class="section border-0 card"
                                data-region="view-overview-messages"
                            >
                                <div id="view-overview-messages-toggle" class="card-header p-0" data-region="toggle">
                                    <button
                                        class="btn btn-link w-100 text-left p-1 p-sm-2 d-flex align-items-center overview-section-toggle collapsed"
                                        data-toggle="collapse"
                                        data-target="#view-overview-messages-target-60bf3510d20bb60bf3510cb0c518"
                                        aria-expanded="false"
                                        aria-controls="view-overview-messages-target-60bf3510d20bb60bf3510cb0c518"
                                    >
                                        <span class="collapsed-icon-container">
                                            <i class="icon fa fa-caret-right fa-fw " aria-hidden="true"  ></i>
                                        </span>
                                        <span class="expanded-icon-container">
                                            <i class="icon fa fa-caret-down fa-fw " aria-hidden="true"  ></i>
                                        </span>
                                        <span class="font-weight-bold">Privado</span>
                                        <small class="hidden ml-1" data-region="section-total-count-container"
                                        aria-label=" total conversations">
                                            (<span data-region="section-total-count"></span>)
                                        </small>
                                        <span class="hidden ml-2" data-region="loading-icon-container">
                                            <span class="loading-icon icon-no-margin"><i class="icon fa fa-circle-o-notch fa-spin fa-fw "  title="Enviando" aria-label="Enviando"></i></span>
                                        </span>
                                        <span class="hidden badge badge-pill badge-primary ml-auto bg-primary"
                                        data-region="section-unread-count"
                                        >
                                            
                                        </span>
                                    </button>
                                </div>
                                                            <div
                                class="collapse border-bottom  lazy-load-list"
                                aria-live="polite"
                                data-region="lazy-load-list"
                                data-user-id="18361061"
                                            id="view-overview-messages-target-60bf3510d20bb60bf3510cb0c518"
            aria-labelledby="view-overview-messages-toggle"
            data-parent="#message-drawer-view-overview-container-60bf3510d20bb60bf3510cb0c518"

                            >
                                
                                <div class="hidden text-center p-2" data-region="empty-message-container">
                                            <p class="text-muted mt-2">No hay conversaciones privadas</p>

                                </div>
                                <div class="hidden list-group" data-region="content-container">
                                    
                                </div>
                                <div class="list-group" data-region="placeholder-container">
                                            <div class="text-center py-2"><span class="loading-icon icon-no-margin"><i class="icon fa fa-circle-o-notch fa-spin fa-fw "  title="Enviando" aria-label="Enviando"></i></span>
</div>

                                </div>
                                <div class="w-100 text-center p-3 hidden" data-region="loading-icon-container" >
                                    <span class="loading-icon icon-no-margin"><i class="icon fa fa-circle-o-notch fa-spin fa-fw "  title="Enviando" aria-label="Enviando"></i></span>
                                </div>
                            </div>
                            </div>
                    </div>
                </div>
                
                <div
                    data-region="view-search"
                    aria-hidden="true"
                    class="h-100 hidden"
                    data-user-id="18361061"
                    data-users-offset="0"
                    data-messages-offset="0"
                    style="overflow-y: auto"
                    
                >
                    <div class="hidden" data-region="search-results-container" style="overflow-y: auto">
                        
                        <div class="d-flex flex-column">
                            <div class="mb-3 bg-white" data-region="all-contacts-container">
                                <div data-region="contacts-container"  class="pt-2">
                                    <h4 class="h6 px-2">Contactos</h4>
                                    <div class="list-group" data-region="list"></div>
                                </div>
                                <div data-region="non-contacts-container" class="pt-2 border-top">
                                    <h4 class="h6 px-2">No contactos</h4>
                                    <div class="list-group" data-region="list"></div>
                                </div>
                                <div class="text-right">
                                    <button class="btn btn-link text-primary" data-action="load-more-users">
                                        <span data-region="button-text">Cargar más</span>
                                        <span data-region="loading-icon-container" class="hidden"><span class="loading-icon icon-no-margin"><i class="icon fa fa-circle-o-notch fa-spin fa-fw "  title="Enviando" aria-label="Enviando"></i></span>
</span>
                                    </button>
                                </div>
                            </div>
                            <div class="bg-white" data-region="messages-container">
                                <h4 class="h6 px-2 pt-2">Mensajes</h4>
                                <div class="list-group" data-region="list"></div>
                                <div class="text-right">
                                    <button class="btn btn-link text-primary" data-action="load-more-messages">
                                        <span data-region="button-text">Cargar más</span>
                                        <span data-region="loading-icon-container" class="hidden"><span class="loading-icon icon-no-margin"><i class="icon fa fa-circle-o-notch fa-spin fa-fw "  title="Enviando" aria-label="Enviando"></i></span>
</span>
                                    </button>
                                </div>
                            </div>
                            <p class="hidden p-3 text-center" data-region="no-results-container">No hay resultados</p>
                        </div>                    </div>
                    <div class="hidden" data-region="loading-placeholder">
                        <div class="text-center pt-3 icon-size-4"><span class="loading-icon icon-no-margin"><i class="icon fa fa-circle-o-notch fa-spin fa-fw "  title="Enviando" aria-label="Enviando"></i></span>
</div>
                    </div>
                    <div class="p-3 text-center" data-region="empty-message-container">
                        <p>Buscar personas y mensajes</p>
                    </div>
                </div>                
                <div class="h-100 hidden bg-white" aria-hidden="true" data-region="view-settings">
                    <div class="hidden" data-region="content-container">
                        
                        <div data-region="settings" class="p-3">
                            <h3 class="h6 font-weight-bold">Privacidad</h3>
                            <p>Puedes restringir quién puede enviarte mensajes</p>
                            <div data-preference="blocknoncontacts" class="mb-3">
                                <fieldset>
                                    <legend class="sr-only">Aceptar mensajes desde:</legend>
                                        <div class="custom-control custom-radio mb-2">
                                            <input
                                                type="radio"
                                                name="message_blocknoncontacts"
                                                class="custom-control-input"
                                                id="block-noncontacts-60bf3510d20bb60bf3510cb0c518-1"
                                                value="1"
                                            >
                                            <label class="custom-control-label ml-2" for="block-noncontacts-60bf3510d20bb60bf3510cb0c518-1">
                                                Únicamente mis contactos
                                            </label>
                                        </div>
                                        <div class="custom-control custom-radio mb-2">
                                            <input
                                                type="radio"
                                                name="message_blocknoncontacts"
                                                class="custom-control-input"
                                                id="block-noncontacts-60bf3510d20bb60bf3510cb0c518-0"
                                                value="0"
                                            >
                                            <label class="custom-control-label ml-2" for="block-noncontacts-60bf3510d20bb60bf3510cb0c518-0">
                                                Mis contactos y cualquier persona de mis cursos
                                            </label>
                                        </div>
                                </fieldset>
                            </div>
                        
                            <div class="hidden" data-region="notification-preference-container">
                                <h3 class="mb-2 mt-4 h6 font-weight-bold">Preferencias de notificación</h3>
                            </div>
                        
                            <h3 class="mb-2 mt-4 h6 font-weight-bold">General</h3>
                            <div data-preference="entertosend">
                                <div class="custom-control custom-switch">
                                    <input type="checkbox" class="custom-control-input" id="enter-to-send-60bf3510d20bb60bf3510cb0c518" >
                                    <label class="custom-control-label" for="enter-to-send-60bf3510d20bb60bf3510cb0c518">
                                        Usar "intro" para enviar
                                    </label>
                                </div>
                            </div>
                        </div>
                    </div>
                    <div data-region="placeholder-container">
                        
                        <div class="d-flex flex-column p-3">
                            <div class="w-25 bg-pulse-grey h6" style="height: 18px"></div>
                            <div class="w-75 bg-pulse-grey mb-4" style="height: 18px"></div>
                            <div class="mb-3">
                                <div class="w-100 d-flex mb-3">
                                    <div class="bg-pulse-grey rounded-circle" style="width: 18px; height: 18px"></div>
                                    <div class="bg-pulse-grey w-50 ml-2" style="height: 18px"></div>
                                </div>
                                <div class="w-100 d-flex mb-3">
                                    <div class="bg-pulse-grey rounded-circle" style="width: 18px; height: 18px"></div>
                                    <div class="bg-pulse-grey w-50 ml-2" style="height: 18px"></div>
                                </div>
                                <div class="w-100 d-flex mb-3">
                                    <div class="bg-pulse-grey rounded-circle" style="width: 18px; height: 18px"></div>
                                    <div class="bg-pulse-grey w-50 ml-2" style="height: 18px"></div>
                                </div>
                            </div>
                            <div class="w-50 bg-pulse-grey h6 mb-3 mt-2" style="height: 18px"></div>
                            <div class="mb-4">
                                <div class="w-100 d-flex mb-2 align-items-center">
                                    <div class="bg-pulse-grey w-25" style="width: 18px; height: 27px"></div>
                                    <div class="bg-pulse-grey w-25 ml-2" style="height: 18px"></div>
                                </div>
                                <div class="w-100 d-flex mb-2 align-items-center">
                                    <div class="bg-pulse-grey w-25" style="width: 18px; height: 27px"></div>
                                    <div class="bg-pulse-grey w-25 ml-2" style="height: 18px"></div>
                                </div>
                            </div>
                            <div class="w-25 bg-pulse-grey h6 mb-3 mt-2" style="height: 18px"></div>
                            <div class="mb-3">
                                <div class="w-100 d-flex mb-2 align-items-center">
                                    <div class="bg-pulse-grey w-25" style="width: 18px; height: 27px"></div>
                                    <div class="bg-pulse-grey w-50 ml-2" style="height: 18px"></div>
                                </div>
                            </div>
                        </div>                    </div>
                </div>            </div>
            <div class="footer-container position-relative" data-region="footer-container">
                
                <div
                    class="hidden border-top bg-white position-relative"
                    aria-hidden="true"
                    data-region="view-conversation"
                    data-enter-to-send="0"
                >
                    <div class="hidden p-sm-2" data-region="content-messages-footer-container">
                        
                            <div
                                class="emoji-auto-complete-container w-100 hidden"
                                data-region="emoji-auto-complete-container"
                                aria-live="polite"
                                aria-hidden="true"
                            >
                            </div>
                        <div class="d-flex mt-sm-1">
                            <textarea
                                dir="auto"
                                data-region="send-message-txt"
                                class="form-control bg-light"
                                rows="3"
                                data-auto-rows
                                data-min-rows="3"
                                data-max-rows="5"
                                aria-label="Escribe un mensaje..."
                                placeholder="Escribe un mensaje..."
                                style="resize: none"
                                maxlength="4096"
                            ></textarea>
                        
                            <div class="position-relative d-flex flex-column">
                                    <div
                                        data-region="emoji-picker-container"
                                        class="emoji-picker-container hidden"
                                        aria-hidden="true"
                                    >
                                        
                                        <div
                                            data-region="emoji-picker"
                                            class="card shadow emoji-picker"
                                        >
                                            <div class="card-header px-1 pt-1 pb-0 d-flex justify-content-between flex-shrink-0">
                                                <button
                                                    class="btn btn-outline-secondary icon-no-margin category-button selected"
                                                    data-action="show-category"
                                                    data-category="Recent"
                                                    title="Reciente"
                                                >
                                                    <i class="icon fa fa-clock-o fa-fw " aria-hidden="true"  ></i>
                                                </button>
                                                <button
                                                    class="btn btn-outline-secondary icon-no-margin category-button"
                                                    data-action="show-category"
                                                    data-category="Smileys & People"
                                                    title="Emoticonos y personas"
                                                >
                                                    <i class="icon fa fa-smile-o fa-fw " aria-hidden="true"  ></i>
                                                </button>
                                                <button
                                                    class="btn btn-outline-secondary icon-no-margin category-button"
                                                    data-action="show-category"
                                                    data-category="Animals & Nature"
                                                    title="Animales y naturaleza"
                                                >
                                                    <i class="icon fa fa-leaf fa-fw " aria-hidden="true"  ></i>
                                                </button>
                                                <button
                                                    class="btn btn-outline-secondary icon-no-margin category-button"
                                                    data-action="show-category"
                                                    data-category="Food & Drink"
                                                    title="Comida y bebida"
                                                >
                                                    <i class="icon fa fa-cutlery fa-fw " aria-hidden="true"  ></i>
                                                </button>
                                                <button
                                                    class="btn btn-outline-secondary icon-no-margin category-button"
                                                    data-action="show-category"
                                                    data-category="Travel & Places"
                                                    title="Viaje y lugares"
                                                >
                                                    <i class="icon fa fa-plane fa-fw " aria-hidden="true"  ></i>
                                                </button>
                                                <button
                                                    class="btn btn-outline-secondary icon-no-margin category-button"
                                                    data-action="show-category"
                                                    data-category="Activities"
                                                    title="Actividades"
                                                >
                                                    <i class="icon fa fa-futbol-o fa-fw " aria-hidden="true"  ></i>
                                                </button>
                                                <button
                                                    class="btn btn-outline-secondary icon-no-margin category-button"
                                                    data-action="show-category"
                                                    data-category="Objects"
                                                    title="Objetos"
                                                >
                                                    <i class="icon fa fa-lightbulb-o fa-fw " aria-hidden="true"  ></i>
                                                </button>
                                                <button
                                                    class="btn btn-outline-secondary icon-no-margin category-button"
                                                    data-action="show-category"
                                                    data-category="Symbols"
                                                    title="Símbolos"
                                                >
                                                    <i class="icon fa fa-heart fa-fw " aria-hidden="true"  ></i>
                                                </button>
                                                <button
                                                    class="btn btn-outline-secondary icon-no-margin category-button"
                                                    data-action="show-category"
                                                    data-category="Flags"
                                                    title="Banderas"
                                                >
                                                    <i class="icon fa fa-flag fa-fw " aria-hidden="true"  ></i>
                                                </button>
                                            </div>
                                            <div class="card-body p-2 d-flex flex-column overflow-hidden">
                                                <div class="input-group mb-1 flex-shrink-0">
                                                    <div class="input-group-prepend">
                                                        <span class="input-group-text pr-0 bg-white text-muted">
                                                            <i class="icon fa fa-search fa-fw " aria-hidden="true"  ></i>
                                                        </span>
                                                    </div>
                                                    <input
                                                        type="text"
                                                        class="form-control border-left-0"
                                                        placeholder="Buscar"
                                                        aria-label="Buscar"
                                                        data-region="search-input"
                                                    >
                                                </div>
                                                <div class="flex-grow-1 overflow-auto emojis-container h-100" data-region="emojis-container">
                                                    <div class="position-relative" data-region="row-container"></div>
                                                </div>
                                                <div class="flex-grow-1 overflow-auto search-results-container h-100 hidden" data-region="search-results-container">
                                                    <div class="position-relative" data-region="row-container"></div>
                                                </div>
                                            </div>
                                            <div
                                                class="card-footer d-flex flex-shrink-0"
                                                data-region="footer"
                                            >
                                                <div class="emoji-preview" data-region="emoji-preview"></div>
                                                <div data-region="emoji-short-name" class="emoji-short-name text-muted text-wrap ml-2"></div>
                                            </div>
                                        </div>
                                    </div>
                                    <button
                                        class="btn btn-link btn-icon icon-size-3 ml-1"
                                        aria-label="Selector de emoticono"
                                        data-action="toggle-emoji-picker"
                                    >
                                        <i class="icon fa fa-smile-o fa-fw " aria-hidden="true"  ></i>
                                    </button>
                                <button
                                    class="btn btn-link btn-icon icon-size-3 ml-1 mt-auto"
                                    aria-label="Enviar mensaje"
                                    data-action="send-message"
                                >
                                    <span data-region="send-icon-container"><i class="icon fa fa-paper-plane fa-fw " aria-hidden="true"  ></i></span>
                                    <span class="hidden" data-region="loading-icon-container"><span class="loading-icon icon-no-margin"><i class="icon fa fa-circle-o-notch fa-spin fa-fw "  title="Enviando" aria-label="Enviando"></i></span>
</span>
                                </button>
                            </div>
                        </div>
                    </div>
                    <div class="hidden p-sm-2" data-region="content-messages-footer-edit-mode-container">
                        
                        <div class="d-flex p-3 justify-content-end">
                            <button
                                class="btn btn-link btn-icon my-1 icon-size-4"
                                data-action="delete-selected-messages"
                                data-toggle="tooltip"
                                data-placement="top"
                                title="Eliminar mensajes seleccionados"
                            >
                                <span data-region="icon-container"><i class="icon fa fa-trash fa-fw " aria-hidden="true"  ></i></span>
                                <span class="hidden" data-region="loading-icon-container"><span class="loading-icon icon-no-margin"><i class="icon fa fa-circle-o-notch fa-spin fa-fw "  title="Enviando" aria-label="Enviando"></i></span>
</span>
                                <span class="sr-only">Eliminar mensajes seleccionados</span>
                            </button>
                        </div>                    </div>
                    <div class="hidden bg-secondary p-sm-3" data-region="content-messages-footer-require-contact-container">
                        
                        <div class="p-3 bg-white">
                            <p data-region="title"></p>
                            <p class="text-muted" data-region="text"></p>
                            <button type="button" class="btn btn-primary btn-block" data-action="request-add-contact">
                                <span data-region="dialogue-button-text">Enviar solicitud de contacto</span>
                                <span class="hidden" data-region="loading-icon-container"><span class="loading-icon icon-no-margin"><i class="icon fa fa-circle-o-notch fa-spin fa-fw "  title="Enviando" aria-label="Enviando"></i></span>
</span>
                            </button>
                        </div>
                    </div>
                    <div class="hidden bg-secondary p-sm-3" data-region="content-messages-footer-require-unblock-container">
                        
                        <div class="p-3 bg-white">
                            <p class="text-muted" data-region="text">Ha bloqueado a este usuario.</p>
                            <button type="button" class="btn btn-primary btn-block" data-action="request-unblock">
                                <span data-region="dialogue-button-text">Desbloquear usuario</span>
                                <span class="hidden" data-region="loading-icon-container"><span class="loading-icon icon-no-margin"><i class="icon fa fa-circle-o-notch fa-spin fa-fw "  title="Enviando" aria-label="Enviando"></i></span>
</span>
                            </button>
                        </div>
                    </div>
                    <div class="hidden bg-secondary p-sm-3" data-region="content-messages-footer-unable-to-message">
                        
                        <div class="p-3 bg-white">
                            <p class="text-muted" data-region="text">No puedes enviar mensajes a este usuario</p>
                        </div>
                    </div>
                    <div class="p-sm-2" data-region="placeholder-container">
                        <div class="d-flex">
                            <div class="bg-pulse-grey w-100" style="height: 80px"></div>
                            <div class="mx-2 mb-2 align-self-end bg-pulse-grey" style="height: 20px; width: 20px"></div>
                        </div>                    </div>
                    <div
                        class="hidden position-absolute"
                        data-region="confirm-dialogue-container"
                        style="top: -1px; bottom: 0; right: 0; left: 0; background: rgba(0,0,0,0.3);"
                    ></div>
                </div>                    <div data-region="view-overview" class="text-center">
                        <a href="https://moodle.upm.es/titulaciones/oficiales/message/index.php">
                            Ver todo
                        </a>
                    </div>
            </div>
        </div>

</div>
    <div id="goto-top-link">
        <a class="btn btn-light" role="button" href="#" aria-label="Go to top">
            <i class="icon fa fa-arrow-up fa-fw " aria-hidden="true"  ></i>
        </a>
    </div>
    <footer id="page-footer" class="py-3 bg-dark text-light">
        <div class="container">
            <div id="course-footer"></div>
    
    
            <div class="logininfo">Usted se ha identificado como <a href="https://moodle.upm.es/titulaciones/oficiales/user/profile.php?id=18361061" title="Ver perfil">MALISEK MIROSLAV</a> (<a href="https://moodle.upm.es/titulaciones/oficiales/login/logout.php?sesskey=FjPb0IgJ50">Cerrar sesión</a>)</div>
            <div class="tool_usertours-resettourcontainer"></div>
            <div class="homelink"><a href="https://moodle.upm.es/titulaciones/oficiales/course/view.php?id=5810">RobotPercComp-GII</a></div>
            <nav class="nav navbar-nav d-md-none" aria-label="Menú personalizado">
                    <ul class="list-unstyled pt-3">
                                        <li>UPM</li>
                                    <li>
                                        <ul class="list-unstyled ml-3">
                                                            <li><a href="https://www.upm.es "target="_blank" title="">Web de la UPM</a></li>
                                                            <li><a href="https://www.upm.es/politecnica_virtual/ "target="_blank" title="">Politécnica Virtual</a></li>
                                                            <li><a href="https://ingenio.upm.es "target="_blank" title="">Biblioteca UPM</a></li>
                                                            <li><a href="https://www.upm.es/webmail_personal/ "target="_blank" title="">Correo electrónico PAS y PDI</a></li>
                                                            <li><a href="https://www.upm.es/webmail_alumnos/ "target="_blank" title="">Correo electrónico Estudiantes</a></li>
                                                            <li><a href="https://moodle.upm.es/titulaciones/propias/" title="">Moodle - Titulaciones propias</a></li>
                                                            <li><a href="https://moodle.upm.es/puestaapunto/" title="">Moodle - Puesta a punto</a></li>
                                                            <li><a href="https://moodle.upm.es/formacion/" title="">Moodle - Formación UPM</a></li>
                                        </ul>
                                    </li>
                                        <li>Ayuda</li>
                                    <li>
                                        <ul class="list-unstyled ml-3">
                                                            <li><a href="https://moodle.upm.es/titulaciones/oficiales/mod/page/view.php?id=867182" title="">Contacto y preguntas frecuentes</a></li>
                                                            <li><a href="https://moodle.upm.es/titulaciones/oficiales/course/view.php?id=242" title="">Ayuda y documentación para profesores</a></li>
                                                            <li><a href="https://moodle.upm.es/titulaciones/oficiales/course/view.php?id=3048" title="">Ayuda y documentación para estudiantes</a></li>
                                                            <li><a href="https://moodle.upm.es/titulaciones/oficiales/course/view.php?id=9106" title="">Curso básico online para el profesor</a></li>
                                                            <li><a href="https://docs.moodle.org/all/es/P%C3%A1gina_Principal" title="">Documentación oficial de Moodle</a></li>
                                                            <li><a href="http://oa.upm.es/65760/7/Manual_Moodle_3_9.pdf" title="">Manual para el profesor</a></li>
                                        </ul>
                                    </li>
                                        <li><a href="#" title="Idioma">Español - Internacional ‎(es)‎</a></li>
                                    <li>
                                        <ul class="list-unstyled ml-3">
                                                            <li><a href="https://moodle.upm.es/titulaciones/oficiales/mod/resource/view.php?id=1141685&amp;lang=de" title="Deutsch ‎(de)‎">Deutsch ‎(de)‎</a></li>
                                                            <li><a href="https://moodle.upm.es/titulaciones/oficiales/mod/resource/view.php?id=1141685&amp;lang=en" title="English ‎(en)‎">English ‎(en)‎</a></li>
                                                            <li><a href="https://moodle.upm.es/titulaciones/oficiales/mod/resource/view.php?id=1141685&amp;lang=es" title="Español - Internacional ‎(es)‎">Español - Internacional ‎(es)‎</a></li>
                                                            <li><a href="https://moodle.upm.es/titulaciones/oficiales/mod/resource/view.php?id=1141685&amp;lang=fr" title="Français ‎(fr)‎">Français ‎(fr)‎</a></li>
                                                            <li><a href="https://moodle.upm.es/titulaciones/oficiales/mod/resource/view.php?id=1141685&amp;lang=it" title="Italiano ‎(it)‎">Italiano ‎(it)‎</a></li>
                                                            <li><a href="https://moodle.upm.es/titulaciones/oficiales/mod/resource/view.php?id=1141685&amp;lang=pt" title="Português - Portugal ‎(pt)‎">Português - Portugal ‎(pt)‎</a></li>
                                                            <li><a href="https://moodle.upm.es/titulaciones/oficiales/mod/resource/view.php?id=1141685&amp;lang=zh_cn" title="简体中文 ‎(zh_cn)‎">简体中文 ‎(zh_cn)‎</a></li>
                                        </ul>
                                    </li>
                    </ul>
            </nav>
            <a href="https://moodle.upm.es/titulaciones/oficiales/mod/page/view.php?id=1045478">Descargar la app para dispositivos móviles</a>
            <script>
//<![CDATA[
var require = {
    baseUrl : 'https://moodle.upm.es/titulaciones/oficiales/lib/requirejs.php/1623142191/',
    // We only support AMD modules with an explicit define() statement.
    enforceDefine: true,
    skipDataMain: true,
    waitSeconds : 0,

    paths: {
        jquery: 'https://moodle.upm.es/titulaciones/oficiales/lib/javascript.php/1623142191/lib/jquery/jquery-3.5.1.min',
        jqueryui: 'https://moodle.upm.es/titulaciones/oficiales/lib/javascript.php/1623142191/lib/jquery/ui-1.12.1/jquery-ui.min',
        jqueryprivate: 'https://moodle.upm.es/titulaciones/oficiales/lib/javascript.php/1623142191/lib/requirejs/jquery-private'
    },

    // Custom jquery config map.
    map: {
      // '*' means all modules will get 'jqueryprivate'
      // for their 'jquery' dependency.
      '*': { jquery: 'jqueryprivate' },
      // Stub module for 'process'. This is a workaround for a bug in MathJax (see MDL-60458).
      '*': { process: 'core/first' },

      // 'jquery-private' wants the real jQuery module
      // though. If this line was not here, there would
      // be an unresolvable cyclic dependency.
      jqueryprivate: { jquery: 'jquery' }
    }
};

//]]>
</script>
<script src="https://moodle.upm.es/titulaciones/oficiales/lib/javascript.php/1623142191/lib/requirejs/require.min.js"></script>
<script>
//<![CDATA[
M.util.js_pending("core/first");require(['core/first'], function() {
require(['core/prefetch']);
;
require(["media_videojs/loader"], function(loader) {
    loader.setUp(function(videojs) {
        videojs.options.flash.swf = "https://moodle.upm.es/titulaciones/oficiales/media/player/videojs/videojs/video-js.swf";
videojs.addLanguage('es', {
  "Play": "Reproducir",
  "Play Video": "Reproducir Vídeo",
  "Pause": "Pausa",
  "Current Time": "Tiempo reproducido",
  "Duration": "Duración total",
  "Remaining Time": "Tiempo restante",
  "Stream Type": "Tipo de secuencia",
  "LIVE": "DIRECTO",
  "Loaded": "Cargado",
  "Progress": "Progreso",
  "Fullscreen": "Pantalla completa",
  "Non-Fullscreen": "Pantalla no completa",
  "Mute": "Silenciar",
  "Unmute": "No silenciado",
  "Playback Rate": "Velocidad de reproducción",
  "Subtitles": "Subtítulos",
  "subtitles off": "Subtítulos desactivados",
  "Captions": "Subtítulos especiales",
  "captions off": "Subtítulos especiales desactivados",
  "Chapters": "Capítulos",
  "You aborted the media playback": "Ha interrumpido la reproducción del vídeo.",
  "A network error caused the media download to fail part-way.": "Un error de red ha interrumpido la descarga del vídeo.",
  "The media could not be loaded, either because the server or network failed or because the format is not supported.": "No se ha podido cargar el vídeo debido a un fallo de red o del servidor o porque el formato es incompatible.",
  "The media playback was aborted due to a corruption problem or because the media used features your browser did not support.": "La reproducción de vídeo se ha interrumpido por un problema de corrupción de datos o porque el vídeo precisa funciones que su navegador no ofrece.",
  "No compatible source was found for this media.": "No se ha encontrado ninguna fuente compatible con este vídeo.",
  "Audio Player": "Reproductor de audio",
  "Video Player": "Reproductor de video",
  "Replay": "Volver a reproducir",
  "Seek to live, currently behind live": "Buscar en vivo, actualmente demorado con respecto a la transmisión en vivo",
  "Seek to live, currently playing live": "Buscar en vivo, actualmente reproduciendo en vivo",
  "Progress Bar": "Barra de progreso",
  "progress bar timing: currentTime={1} duration={2}": "{1} de {2}",
  "Descriptions": "Descripciones",
  "descriptions off": "descripciones desactivadas",
  "Audio Track": "Pista de audio",
  "Volume Level": "Nivel de volumen",
  "The media is encrypted and we do not have the keys to decrypt it.": "El material audiovisual está cifrado y no tenemos las claves para descifrarlo.",
  "Close": "Cerrar",
  "Modal Window": "Ventana modal",
  "This is a modal window": "Esta es una ventana modal",
  "This modal can be closed by pressing the Escape key or activating the close button.": "Esta ventana modal puede cerrarse presionando la tecla Escape o activando el botón de cierre.",
  ", opens captions settings dialog": ", abre el diálogo de configuración de leyendas",
  ", opens subtitles settings dialog": ", abre el diálogo de configuración de subtítulos",
  ", selected": ", seleccionado",
  "Close Modal Dialog": "Cierra cuadro de diálogo modal",
  ", opens descriptions settings dialog": ", abre el diálogo de configuración de las descripciones",
  "captions settings": "configuración de leyendas",
  "subtitles settings": "configuración de subtítulos",
  "descriptions settings": "configuración de descripciones",
  "Text": "Texto",
  "White": "Blanco",
  "Black": "Negro",
  "Red": "Rojo",
  "Green": "Verde",
  "Blue": "Azul",
  "Yellow": "Amarillo",
  "Magenta": "Magenta",
  "Cyan": "Cian",
  "Background": "Fondo",
  "Window": "Ventana",
  "Transparent": "Transparente",
  "Semi-Transparent": "Semitransparente",
  "Opaque": "Opaca",
  "Font Size": "Tamaño de fuente",
  "Text Edge Style": "Estilo de borde del texto",
  "None": "Ninguno",
  "Raised": "En relieve",
  "Depressed": "Hundido",
  "Uniform": "Uniforme",
  "Dropshadow": "Sombra paralela",
  "Font Family": "Familia de fuente",
  "Proportional Sans-Serif": "Sans-Serif proporcional",
  "Monospace Sans-Serif": "Sans-Serif monoespacio",
  "Proportional Serif": "Serif proporcional",
  "Monospace Serif": "Serif monoespacio",
  "Casual": "Informal",
  "Script": "Cursiva",
  "Small Caps": "Minúsculas",
  "Reset": "Restablecer",
  "restore all settings to the default values": "restablece todas las configuraciones a los valores predeterminados",
  "Done": "Listo",
  "Caption Settings Dialog": "Diálogo de configuración de leyendas",
  "Beginning of dialog window. Escape will cancel and close the window.": "Comienzo de la ventana de diálogo. La tecla Escape cancelará la operación y cerrará la ventana.",
  "End of dialog window.": "Final de la ventana de diálogo.",
  "{1} is loading.": "{1} se está cargando."
});

    });
});;
function legacy_activity_onclick_handler_1(e) { e.halt(); window.open('https://moodle.upm.es/titulaciones/oficiales/mod/resource/view.php?id=1121276&redirect=1'); return false; };
function legacy_activity_onclick_handler_2(e) { e.halt(); window.open('https://moodle.upm.es/titulaciones/oficiales/mod/resource/view.php?id=1349338&redirect=1'); return false; };
function legacy_activity_onclick_handler_3(e) { e.halt(); window.open('https://moodle.upm.es/titulaciones/oficiales/mod/resource/view.php?id=1345092&redirect=1'); return false; };
function legacy_activity_onclick_handler_4(e) { e.halt(); window.open('https://moodle.upm.es/titulaciones/oficiales/mod/resource/view.php?id=1141685&redirect=1'); return false; };

require(['jquery', 'core/custom_interaction_events'], function($, CustomEvents) {
    CustomEvents.define('#single_select60bf3510cb0c53', [CustomEvents.events.accessibleChange]);
    $('#single_select60bf3510cb0c53').on(CustomEvents.events.accessibleChange, function() {
        var ignore = $(this).find(':selected').attr('data-ignore');
        if (typeof ignore === typeof undefined) {
            $('#single_select_f60bf3510cb0c52').submit();
        }
    });
});
;

require(['jquery', 'message_popup/notification_popover_controller'], function($, controller) {
    var container = $('#nav-notification-popover-container');
    var controller = new controller(container);
    controller.registerEventListeners();
    controller.registerListNavigationEventListeners();
});
;

require(
[
    'jquery',
    'core_message/message_popover'
],
function(
    $,
    Popover
) {
    var toggle = $('#message-drawer-toggle-60bf3510ced9060bf3510cb0c58');
    Popover.init(toggle);
});
;

        require(['jquery', 'core/custom_interaction_events'], function($, CustomEvents) {
            CustomEvents.define('#jump-to-activity', [CustomEvents.events.accessibleChange]);
            $('#jump-to-activity').on(CustomEvents.events.accessibleChange, function() {
                if (!$(this).val()) {
                    return false;
                }
                $('#url_select_f60bf3510cb0c517').submit();
            });
        });
    ;

require(['jquery', 'core_message/message_drawer'], function($, MessageDrawer) {
    var root = $('#message-drawer-60bf3510d20bb60bf3510cb0c518');
    MessageDrawer.init(root, '60bf3510d20bb60bf3510cb0c518', false);
});
;

require(['jquery', 'core/custom_interaction_events'], function($, CustomEvents) {
    CustomEvents.define('#single_select60bf3510cb0c520', [CustomEvents.events.accessibleChange]);
    $('#single_select60bf3510cb0c520').on(CustomEvents.events.accessibleChange, function() {
        var ignore = $(this).find(':selected').attr('data-ignore');
        if (typeof ignore === typeof undefined) {
            $('#single_select_f60bf3510cb0c519').submit();
        }
    });
});
;

M.util.js_pending('theme_boost/loader');
require(['theme_boost/loader'], function() {
    M.util.js_complete('theme_boost/loader');
});

M.util.js_pending('theme_boost/drawer');
require(['theme_boost/drawer'], function(drawer) {
    drawer.init();
    M.util.js_complete('theme_boost/drawer');
});
;
M.util.js_pending('core/notification'); require(['core/notification'], function(amd) {amd.init(8143720, [], true); M.util.js_complete('core/notification');});;
M.util.js_pending('core/log'); require(['core/log'], function(amd) {amd.setConfig({"level":"warn"}); M.util.js_complete('core/log');});;
M.util.js_pending('core/page_global'); require(['core/page_global'], function(amd) {amd.init(); M.util.js_complete('core/page_global');});M.util.js_complete("core/first");
});
//]]>
</script>
<script>
//<![CDATA[
M.str = {"moodle":{"lastmodified":"\u00daltima modificaci\u00f3n","name":"Nombre","error":"Error","info":"Informaci\u00f3n","yes":"S\u00ed","no":"No","ok":"OK","cancel":"Cancelar","confirm":"Confirmar","areyousure":"\u00bfEst\u00e1 seguro?","closebuttontitle":"Cerrar","unknownerror":"Error desconocido","file":"Archivo","url":"URL"},"repository":{"type":"Tipo","size":"Tama\u00f1o","invalidjson":"Cadena JSON no v\u00e1lida","nofilesattached":"No se han adjuntado archivos","filepicker":"Selector de archivos","logout":"Salir","nofilesavailable":"No hay archivos disponibles","norepositoriesavailable":"Lo sentimos, ninguno de sus repositorios actuales puede devolver archivos en el formato solicitado.","fileexistsdialogheader":"El archivo existe","fileexistsdialog_editor":"Un archivo con ese nombre ha sido anexado al texto que Usted est\u00e1 editando","fileexistsdialog_filemanager":"Ya ha sido anexado un archivo con ese nombre","renameto":"Cambiar el nombre a \"{$a}\"","referencesexist":"Existen {$a} archivos de alias\/atajos que emplean este archivo como su or\u00edgen","select":"Seleccionar"},"admin":{"confirmdeletecomments":"Est\u00e1 a punto de eliminar comentarios, \u00bfest\u00e1 seguro?","confirmation":"Confirmaci\u00f3n"},"debug":{"debuginfo":"Informaci\u00f3n de depuraci\u00f3n","line":"L\u00ednea","stacktrace":"Trazado de la pila (stack)"},"langconfig":{"labelsep":":"}};
//]]>
</script>
<script>
//<![CDATA[
(function() {Y.use("moodle-filter_mathjaxloader-loader",function() {M.filter_mathjaxloader.configure({"mathjaxconfig":"MathJax.Hub.Config({\r\n    config: [\"Accessible.js\", \"Safe.js\"],\r\n    errorSettings: { message: [\"!\"] },\r\n    skipStartupTypeset: true,\r\n    messageStyle: \"none\"\r\n});\r\n","lang":"es"});
});
Y.use("moodle-filter_glossary-autolinker",function() {M.filter_glossary.init_filter_autolinking({"courseid":0});
});
M.util.help_popups.setup(Y);
 M.util.js_pending('random60bf3510cb0c521'); Y.on('domready', function() { M.util.js_complete("init");  M.util.js_complete('random60bf3510cb0c521'); });
})();
//]]>
</script>

        </div>
    </footer>
</div>

</body>
</html>