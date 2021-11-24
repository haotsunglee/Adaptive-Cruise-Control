function CodeMetrics() {
	 this.metricsArray = {};
	 this.metricsArray.var = new Array();
	 this.metricsArray.fcn = new Array();
	 this.metricsArray.var["rs"] = {file: "N:\\Final_Project\\Project_Files\\path_data.c",
	size: 1643};
	 this.metricsArray.fcn["Pick_lead_Outputs_wrapper"] = {file: "N:\\Final_Project\\Project_Files\\Pick_lead_wrapper.c",
	stack: 8,
	stackTotal: 8};
	 this.metricsArray.fcn["bicycle_model_Outputs_wrapper"] = {file: "N:\\Final_Project\\Project_Files\\bicycle_model_wrapper.c",
	stack: 0,
	stackTotal: 0};
	 this.metricsArray.fcn["calc_us_Outputs_wrapper"] = {file: "N:\\Final_Project\\Project_Files\\calc_us_wrapper.c",
	stack: 0,
	stackTotal: 0};
	 this.metricsArray.fcn["cos"] = {file: "C:\\Program Files\\MATLAB\\R2020a\\sys\\lcc\\include\\math.h",
	stack: 0,
	stackTotal: 0};
	 this.metricsArray.fcn["sin"] = {file: "C:\\Program Files\\MATLAB\\R2020a\\sys\\lcc\\include\\math.h",
	stack: 0,
	stackTotal: 0};
	 this.getMetrics = function(token) { 
		 var data;
		 data = this.metricsArray.var[token];
		 if (!data) {
			 data = this.metricsArray.fcn[token];
			 if (data) data.type = "fcn";
		 } else { 
			 data.type = "var";
		 }
	 return data; }; 
	 this.codeMetricsSummary = '<a href="Final_project_template_metrics.html">Global Memory: 1643(bytes) Maximum Stack: 8(bytes)</a>';
	}
CodeMetrics.instance = new CodeMetrics();
